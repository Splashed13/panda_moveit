// Include the header file for the "pick_and_place" functionality
#include "../include/panda_moveit/pick_and_place.hpp"

// Define the namespace to group related code together
namespace pnp
{
    PickandPlace::PickandPlace(ros::NodeHandle &nh) : move_group_interface_arm(PLANNING_GROUP_ARM), move_group_interface_gripper(PLANNING_GROUP_GRIPPER)
    {
        // Set up a publisher for the pose point visualization marker
        pose_point_pub = nh.advertise<visualization_msgs::Marker>("pose_point", 10);

        // Set the planning time for the arm movement
        const double PLANNING_TIME = 15.0;
        move_group_interface_arm.setPlanningTime(PLANNING_TIME);

        // Use raw pointers for performance. These point to the joint model groups for the arm and gripper
        joint_model_group_arm = move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
        joint_model_group_gripper = move_group_interface_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
    }

    void PickandPlace::writeRobotDetails()
    {
        // Print out the planning frame for the arm
        ROS_INFO_NAMED("pnp", "Planning frame: %s", move_group_interface_arm.getPlanningFrame().c_str());

        // Get the link names for the arm and concatenate them into a single string, then print
        std::vector<std::string> linkNames = move_group_interface_arm.getLinkNames();
        std::string linkNamesArm = boost::algorithm::join(linkNames, ", ");
        ROS_INFO_NAMED("pnp", "Arm links: %s", linkNamesArm.c_str());

        // Get the joint names for the arm and concatenate them into a single string, then print
        std::vector<std::string> jointNamesArm = move_group_interface_arm.getJoints();
        std::string jointNamesArmString = boost::algorithm::join(jointNamesArm, ", ");
        ROS_INFO_NAMED("pnp", "Arm joint names: %s", jointNamesArmString.c_str());

        // Get the joint names for the gripper and concatenate them into a single string, then print
        std::vector<std::string> jointNamesGripper = move_group_interface_gripper.getJoints();
        std::string jointNamesGripperString = boost::algorithm::join(jointNamesGripper, ", ");
        ROS_INFO_NAMED("pnp", "Gripper joints names: %s", jointNamesGripperString.c_str());

        // Get the list of all available planning groups, concatenate them into a single string, then print
        std::vector<std::string> planningGroups = move_group_interface_arm.getJointModelGroupNames();
        std::string planningGroupsString = boost::algorithm::join(planningGroups, ", ");
        ROS_INFO_NAMED("pnp", "Available Planning Groups: %s", planningGroupsString.c_str());

    
        // Get the home joint values and print them out
        home_joint_values = move_group_interface_arm.getCurrentJointValues();
        std::string jointValuesString;
        for (const auto &joint : home_joint_values)
        {
            jointValuesString += std::to_string(joint) + ", ";
        }
        ROS_INFO_NAMED("pnp", "Home joint values: %s", jointValuesString.c_str());

    }


    void PickandPlace::createCollisionObject(std::string id, std::vector<double> dimensions, std::vector<double> position, double rotation_z)
    {
        // Create collision object
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        collision_object.id = id;

        // Convert rotation to quaternion
        double rotation_radians = rotation_z * M_PI / 180.0;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, rotation_radians);

        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = dimensions[0];
        primitive.dimensions[primitive.BOX_Y] = dimensions[1];
        primitive.dimensions[primitive.BOX_Z] = dimensions[2];

        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.position.x = position[0];
        box_pose.position.y = position[1];
        box_pose.position.z = position[2];
        box_pose.orientation = tf2::toMsg(quaternion);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object); 

        // Add the collision object into the world
        // (using a vector that could contain additional objects)
        planning_scene_interface.addCollisionObjects(collision_objects);

        // ROS_INFO_NAMED added object frame.id to world
        ROS_INFO_NAMED("pnp", "Added %s into the world", id.c_str());
    }

    
    void PickandPlace::createCollisionScene()
    {
        // Floor
        // std::vector<double> floor_dimensions = {2.5, 2.5, 0.01};
        // std::vector<double> floor_position = {0.0, 0.0, -0.01};
        // createCollisionObject("floor", floor_dimensions, floor_position, 0.0);

        // Box 1
        std::vector<double> box1_dimensions = {0.2, 0.4, box1.at("z_height")};
        std::vector<double> box1_position = {box1.at("x_pos"), box1.at("y_pos"), box1.at("z_height") / 2.0};
        createCollisionObject("box1", box1_dimensions, box1_position, 0.0);

        // // Box 2
        // std::vector<double> box2_dimensions = {0.3, 0.2, box2.at("z_height")};
        // std::vector<double> box2_position = {box2.at("x_pos"), box2.at("y_pos"), box2.at("z_height") / 2};
        // createCollisionObject("box2", box2_dimensions, box2_position, 0.0);

        // Rod
        std::vector<double> rod_dimensions = {0.02, 0.02, rod_height};
        std::vector<double> rod_position = {box1.at("x_pos"), box1.at("y_pos"), rod_height / 2.0 + box1.at("z_height")};
        createCollisionObject("rod", rod_dimensions, rod_position, 45.0);
    }

    void PickandPlace::clean_scene()
    {
        std::vector<std::string> object_ids;
        object_ids.push_back("floor");
        object_ids.push_back("box1");
        object_ids.push_back("box2");
        object_ids.push_back("rod");
        planning_scene_interface.removeCollisionObjects(object_ids);
    }


    void PickandPlace::set_pose_target(std::vector<double> translation, std::vector<double> rotation)
    {
        // Euler angles to radians
        std::vector<double> rotation_rads = {
            rotation[0] * M_PI / 180.0,
            rotation[1] * M_PI / 180.0,
            rotation[2] * M_PI / 180.0};

        Eigen::Matrix4d homogeneous_mat_arm = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d Rz1, Ry, Rz2;
        Rz1 = (Eigen::AngleAxisd(rotation_rads[0], Eigen::Vector3d::UnitZ())).toRotationMatrix();
        Ry = (Eigen::AngleAxisd(rotation_rads[1], Eigen::Vector3d::UnitY())).toRotationMatrix();
        Rz2 = (Eigen::AngleAxisd(rotation_rads[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
        Eigen::Matrix3d R = Rz1 * Ry * Rz2;
        homogeneous_mat_arm.block<3, 3>(0, 0) = R;
        homogeneous_mat_arm(0, 3) = translation[0];
        homogeneous_mat_arm(1, 3) = translation[1];
        homogeneous_mat_arm(2, 3) = translation[2];

        // Create a homogeneous transformation matrix for the end effector with no rotation
        Eigen::Matrix4d homogeneous_trans_end_effector = Eigen::Matrix4d::Identity();
        homogeneous_trans_end_effector(2, 3) = end_effector_palm_length;

        // Multiply the homogeneous transformation matrix of the arm by the inverse of the homogeneous transformation matrix of the end effector
        Eigen::Matrix4d homogeneous_mat = homogeneous_mat_arm * homogeneous_trans_end_effector.inverse();

        // Create a quaternion from euler angles
        tf2::Quaternion quaternion;
        // get a quaternion from the rotation matrix R
        Eigen::Quaterniond eigen_quat(R);
        // convert eigen quaternion to tf quaternion
        tf2::convert(tf2::toMsg(eigen_quat), quaternion);

        // Create message types for the pose target
        geometry_msgs::Quaternion orientation;
        orientation = tf2::toMsg(quaternion);

        geometry_msgs::Point position;
        position.x = homogeneous_mat(0, 3);
        position.y = homogeneous_mat(1, 3);
        position.z = homogeneous_mat(2, 3);

        // Set the target pose message
        geometry_msgs::Pose pose_target;
        pose_target.position = position;
        pose_target.orientation = orientation;

        // add pose arrow
        add_pose_arrow(pose_target.position, rotation_rads[2]);

        // set the pose target
        move_group_interface_arm.setPoseTarget(pose_target);

        // move to arm to the target pose
        bool success = (move_group_interface_arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // print if the arm was able to move to the target pose
        ROS_INFO_NAMED("pnp", "Moving to pose target %s", success ? "" : "FAILED");
    }

    void PickandPlace::add_pose_arrow(geometry_msgs::Point desired_position, float z_rotation)
    {
        // Publish a marker at the desired pose
        visualization_msgs::Marker marker;
        marker.ns = "arrow";
        marker.id = 0;
        marker.header.frame_id = "panda_link0";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = end_effector_palm_length;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, z_rotation);
        geometry_msgs::Pose Pose;
        Pose.position = desired_position;
        Pose.orientation = tf2::toMsg(quaternion);
        marker.pose = Pose;

        // Publish the marker
        pose_point_pub.publish(marker);

        // print pose arrow successfully published
        ROS_INFO_NAMED("pnp", "Pose arrow successfully published");
    }

    void PickandPlace::remove_pose_arrow()
    {
        visualization_msgs::Marker marker;
        marker.ns = "arrow";
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETE;

        // Publish the marker
        pose_point_pub.publish(marker);

        // print pose arrow successfully removed
        ROS_INFO_NAMED("pnp", "Pose arrow successfully removed");
    }


    std::vector<double> PickandPlace::get_rod_position()
    {
        auto object_poses = planning_scene_interface.getObjectPoses({"rod"});
        if (object_poses.count("rod") == 0)
        {
            throw std::runtime_error("Rod not found in object_poses");
        }

        auto rod_pose = object_poses["rod"];
        return {rod_pose.position.x, rod_pose.position.y, rod_pose.position.z};
    }

    void PickandPlace::go_to_home_position()
    {
        // uses the home_joint_value variable to go to the home position of the robot
        move_group_interface_arm.setJointValueTarget(home_joint_values);

        // move to arm to the target pose
        bool success = (move_group_interface_arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // print if the arm was able to move to the target pose
        ROS_INFO_NAMED("pnp", "Moving to home position %s", success ? "" : "FAILED");

        // move the arm to the target pose
        move_group_interface_arm.move();
    
    }

    void PickandPlace::open_gripper(){
        // Set the joint value target for the gripper
        move_group_interface_gripper.setJointValueTarget(OPEN_GRIPPER);
        bool success = (move_group_interface_gripper.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("pnp", "Opening gripper %s", success ? "" : "FAILED");
        move_group_interface_gripper.move();

    }


    void PickandPlace::run()
    {
        // Write robot details
        writeRobotDetails();

        // open gripper
        open_gripper();

        // Create collision scene
        createCollisionScene();

        // get rod position
        std::vector<double> rod_position = get_rod_position();

        //std::vector<double> rod_position = {box1.at("x_pos"), box1.at("y_pos"), rod_height / 2.0 + box1.at("z_height")};
        //ROS_INFO_NAMED("pnp", "Rod position: %f, %f, %f", rod_position[0], rod_position[1], rod_position[2]);
        
        // set pose target
        set_pose_target(rod_position, {45, 90, 45});

        // execute the plan
        move_group_interface_arm.move();

        std::cout << "Press enter to continue...";
        std::cin.ignore();

        // reset
        go_to_home_position();
        clean_scene();
        remove_pose_arrow();

        std::cout << "Press enter to exit...";
        std::cin.ignore();
        
    }
};