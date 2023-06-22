// ROS
#include <ros/ros.h>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <boost/algorithm/string/join.hpp>

namespace pnp
{
    // This class contains all the functions needed to perform the pick and place operation
    class PickandPlace
    {
    private:
        ros::Publisher pose_point_pub;
        // Set global parameters of panda arm
        const std::vector<double> OPEN_GRIPPER = {0.035, 0.035};
        const std::vector<double> CLOSE_GRIPPER = {0.011, 0.011};
        const double end_effector_palm_length = 0.058 * 1.8; // 1.4 is padding

        // Map is the python equivalent of a dictionary
        const std::map<std::string, double> box1 = {
            {"x_pos", 0.6}, {"y_pos", 0.2}, {"z_height", 0.2}};
        const std::map<std::string, double> box2 = {
            {"x_pos", 0}, {"y_pos", 0.6}, {"z_height", 0.1}};
        const double rod_height = 0.2;

        // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
        // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
        // are used interchangeably.
        const std::string PLANNING_GROUP_ARM = "panda_arm";
        const std::string PLANNING_GROUP_GRIPPER = "panda_hand";

        moveit::planning_interface::MoveGroupInterface move_group_interface_arm;
        moveit::planning_interface::MoveGroupInterface move_group_interface_gripper;

        // Create a moveit::planning_interface::MoveGroupInterface::Plan object to store the movements
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // planning_scene_interface allows us to add and remove collision objects in the world
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const robot_state::JointModelGroup *joint_model_group_arm;
        const robot_state::JointModelGroup *joint_model_group_gripper;

        std::vector<double> floor_dimensions = {2.5, 2.5, 0.01};
        std::vector<double> floor_position = {0.0, 0.0, -0.01};

        std::vector<double> box1_dimensions = {0.2, 0.4, box1.at("z_height")};
        std::vector<double> box1_position = {box1.at("x_pos"), box1.at("y_pos"), box1.at("z_height") / 2.0};

        // empty vector to be filled with the joint values of the robot
        std::vector<double> home_joint_values;

    public:
        /**
         * @brief Constructs a new PickandPlace object.
         *
         * Initializes the arm and gripper interfaces, and sets up the publisher for pose_point. 
         * It also sets the planning time for the arm movement and assigns the joint model groups for the arm and gripper.
         * 
         * @param nh Reference to the NodeHandle object to handle interaction with the ROS system, such as creating publishers.
         */
        PickandPlace(ros::NodeHandle &nh);

        /**
         * @brief Writes and logs the details about the robot arm and gripper, including link and joint names, 
         * available planning groups, and the home joint values.
         *
         * The details are logged to ROS_INFO.
         *
         * This method retrieves the following information:
         * 1. The planning frame for the arm.
         * 2. The names of all the links in the arm.
         * 3. The names of all the joints in the arm.
         * 4. The names of all the joints in the gripper.
         * 5. The list of all available planning groups.
         * 6. The home joint values for the arm.
         */
        void writeRobotDetails(void);

        /**
         * @brief Creates a collision object in the planning scene.
         *
         * @param id The ID to assign to the object.
         * @param dimensions A vector containing the dimensions of the object in the order: x, y, z.
         * @param position A vector containing the position of the object in the order: x, y, z.
         * @param rotation_z The rotation of the object around the z-axis in degrees.
         */
        void createCollisionObject(std::string id, std::vector<double> dimensions, std::vector<double> position, double rotation_z);
        
        /**
         * @brief Creates a collision scene including the floor and various objects.
         */
        void createCollisionScene(void);

        /**
         * @brief Clears all the objects in the planning scene.
         */
        void clean_scene(void);

        /**
         * @brief Sets the pose target for the end effector given the desired position and rotation.
         *
         * @param translation A vector containing the desired position of the end effector in the order: x, y, z.
         * @param rotation A vector containing the desired rotation of the end effector in Euler angles ZYZ in degrees.
         */
        void set_pose_target(std::vector<double> translation, std::vector<double> rotation);

        /**
         * @brief Adds a pose arrow marker at the desired position with the given rotation.
         *
         * @param desired_position The position of the arrow.
         * @param z_rotation The rotation of the arrow around the z-axis in radians.
         */
        void add_pose_arrow(geometry_msgs::Point desired_position, float z_rotation);


        /**
         * @brief Deletes the visualization marker (pose arrow) previously added.
         * 
         * The method creates a Marker message with the DELETE action and the namespace and ID of the marker to be deleted. 
         * The marker is then published, which removes the visualization marker from the visualization.
         * 
         * @warning It's assumed that the marker to be deleted is always in the "arrow" namespace with ID 0.
         */
        void remove_pose_arrow(void);

        /**
         * @brief Retrieves the position of the rod object from the planning scene.
         *
         * @return A vector containing the position of the rod in the order: x, y, z.
         * @throws std::runtime_error If the rod cannot be found in the planning scene.
         */
        std::vector<double> get_rod_position(void);

        /**
         * @brief Moves the robot arm to its home position.
         * 
         * This function uses MoveIt! to plan and execute a trajectory that moves the robot arm 
         * from its current position to its predefined home position. The home position is defined by the `home_joint_values` member variable.
         * A message is printed to the ROS console indicating whether the trajectory planning was successful or not.
         * If the plan is successful, the robot arm is then moved to the home position.
         * 
         * @note This function does not take any parameters and does not return a value.
         */
        void go_to_home_position(void);

        void open_gripper(void);

        /**
         * @brief Runs the pick-and-place procedure for the robot.
         * 
         * This function is the main driver of the robot's operation. It begins by printing the robot's details to the ROS console,
         * then sets up the collision scene for planning purposes. 
         * 
         * It retrieves the current position of the rod object from the environment and sets this as the target pose for the robot arm, 
         * with a specified orientation in Euler angles (ZYZ convention).
         * 
         * The robot arm is then moved to the target pose and the user is prompted to press enter to continue.
         * 
         * Once the user presses enter, the robot arm is moved back to its home position and the collision objects are removed from the scene.
         * 
         * Finally, the user is prompted to press enter to exit the program.
         * 
         * @note This function does not take any parameters and does not return a value.
         */
        void run(void);

    };
};
