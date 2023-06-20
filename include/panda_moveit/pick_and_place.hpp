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
        PickandPlace(ros::NodeHandle &nh);
        void writeRobotDetails(void);
        void createCollisionObject(std::string id, std::vector<double> dimensions, std::vector<double> position, double rotation_z);
        void createCollisionScene(void);
        void clean_scene(void);
        void set_pose_target(std::vector<double> translation, std::vector<double> rotation);
        void add_pose_arrow(geometry_msgs::Point desired_position, float z_rotation);
        std::vector<double> get_rod_position(void);
        void pick(void);
        void run(void);
    };
};
