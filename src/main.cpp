#include "panda_moveit/pick_and_place.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle nh;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(5);
    spinner.start();

    // add a short sleep so the node can finish initializing
    ros::Duration(0.5).sleep();

    // Instantiate the PickandPlace class
    pnp::PickandPlace pnp(nh);

    // Run pick and place operations
    pnp.run();

    // Shutdown the node and join the thread back before exiting
    ros::shutdown();

    return 0;
};