#!/usr/bin/env python

# Import libraries
import rospy
import moveit_commander
import sys
import math
import numpy as np
from tf.transformations import quaternion_from_euler, euler_matrix, translation_matrix

# Import messages
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import moveit_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg
import std_msgs.msg

# Global variables
OPEN_GRIPPER = [0.035, 0.035]
CLOSE_GRIPPER = [0.011, 0.011]
end_effector_palm_length = 0.058 * 1.8  # 1.4 is padding

box1 = {"x_pos": 0.6, "y_pos": 0.2, "z_height": 0.2}
box2 = {"x_pos": 0, "y_pos": 0.6, "z_height": 0.1}
# rod parameters
rod_height = 0.2


class PickAndPlace(object):
    """Pick and place a box using moveit_commander"""

    def __init__(self):
        super(PickAndPlace, self).__init__()
        rospy.init_node("pnp_pipeline")

        # Initialize moveit_commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
    
        self.pose_point_pub = rospy.Publisher(
            "pose_point", visualization_msgs.msg.Marker, queue_size=10
        )
        # Create a RobotCommander object
        robot = moveit_commander.RobotCommander()
        # Create a PlanningSceneInterface object
        scene = moveit_commander.PlanningSceneInterface()
        # Create a MoveGroupCommander object for the panda arm
        arm = moveit_commander.MoveGroupCommander("panda_arm")
        # Set the number of planning attempts to 120
        arm.set_num_planning_attempts(120)
        # Set the planning time to 10 seconds
        arm.set_planning_time(10)
        # End effector group
        end_effector = moveit_commander.MoveGroupCommander("panda_hand")
        # Get the reference frame for the robot
        planning_frame = arm.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        # Get the end-effector link for the group
        eef_link = arm.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        # Get a list of all the groups in the robot
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        # Print the entire state of the robot for debugging purposes
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.robot = robot
        self.scene = scene
        self.arm = arm
        self.end_effector = end_effector
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    ## SCENE METHODS
    def create_collision_scene(self) -> None:
        """Create a collision scene"""
        # add floor, box1, box2 and rod to the scene
        self.scene.add_object(
            self.create_collision_object("floor", [2.5, 2.5, 0.01], [0, 0, -0.01], 0)
        )
        self.scene.add_object(
            self.create_collision_object(
                "box1",
                [0.2, 0.4, box1["z_height"]],
                [box1["x_pos"], box1["y_pos"], box1["z_height"] / 2],
                0,
            )
        )
        self.scene.add_object(
            self.create_collision_object(
                "box2",
                [0.3, 0.2, box2["z_height"]],
                [box2["x_pos"], box2["y_pos"], box2["z_height"] / 2],
                0,
            )
        )
        self.scene.add_object(
            self.create_collision_object(
                "rod",
                [0.02, 0.02, rod_height],
                [box1["x_pos"], box1["y_pos"], rod_height / 2 + box1["z_height"]],
                45,
            )
        )

    def create_collision_object(
        self, id: str, dimensions: list, position: list, rotation_z: int
    ) -> moveit_msgs.msg.CollisionObject:
        """Create collision objects and return it."""
        object = moveit_msgs.msg.CollisionObject()
        object.header.frame_id = (
            self.planning_frame
        )  # "panda_link0" satisfies the std_msgs.msg.Header requirements
        object.id = id

        solid = shape_msgs.msg.SolidPrimitive()
        solid.type = solid.BOX
        solid.dimensions = dimensions
        object.primitives = [solid]

        # rotation about the z axis
        rotation = [0, 0, math.radians(rotation_z)]
        quaternion = quaternion_from_euler(
            rotation[0], rotation[1], rotation[2], axes="szyz"
        )

        # Position messages initailized and assigned
        object_point = Point(position[0], position[1], position[2])
        orientation = Quaternion(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )
        object_pose = Pose(object_point, orientation)

        object.primitive_poses = [object_pose]
        object.operation = object.ADD
        return object

    def clean_scene(self) -> None:
        """Clean the scene"""
        self.scene.remove_world_object("floor")
        self.scene.remove_world_object("box1")
        self.scene.remove_world_object("box2")
        self.scene.remove_world_object("rod")

    ## MOVEMENT METHODS
    def go_to_joint_state(self, joint_goal) -> None:
        """Go to a specified joint state"""
        self.arm.go(joint_goal, wait=True)
        self.arm.stop()

    def go_to_pose_target(
        self, translation: list, rotation: list, relative: bool = False
    ) -> None:
        """Go to a specified pose, the pose is selected by the user as the desired location
        of the palm of the hand. The orientation is specified by the user as a list of Euler angles
        and the rotation in which they are applied (static or relative needs to be specified in the
        axis=<order> flag of the euler tf functions)."""

        # static- is rotation based on the world frame axis (x,y,z)- euler rotation zyz
        if relative is False:
            # Set the target orientation initalize position and orientation messages
            rotation_rads = [
                math.radians(rotation[0]),
                math.radians(rotation[1]),
                math.radians(rotation[2]),
            ]
            homogeneous_mat_arm = euler_matrix(
                rotation_rads[0], rotation_rads[1], rotation_rads[2], axes="szyz"
            )
            # add the translation to the homogeneous transformation matrix
            homogeneous_mat_arm[:3, 3] = translation[:3]   # w^T_ee 
            # print the homogeneous transformation matrix for debugging purposes
            #print("homogeneous_mat_arm: ", homogeneous_mat_arm)


            # create a homogeneous transformation matrix for the end effector with no rotation
            homogeneous_trans_end_effector = translation_matrix( # palm^T_ee     <- thus this gets inversed
                [0, 0, end_effector_palm_length]
            )

            # print the homogeneous transformation matrix for debugging purposes
            #print("homogenous translation end effector matrix: ", homogeneous_trans_end_effector)


            # multiply the homogeneous transformation matrix of the arm by the inverse of the homogeneous transformation matrix of the end effector
            homogeneous_mat = np.dot( # w^T_ee * (palm^T_ee)^-1 = w^T_palm
                homogeneous_mat_arm, np.linalg.inv(homogeneous_trans_end_effector)
            )

            # print the homogeneous transformation matrix for debugging purposes
            #print("homogeneous_mat: ", homogeneous_mat)


            # quaternion_from_euler using input rotation values
            quaternion = quaternion_from_euler(
                rotation_rads[0], rotation_rads[1], rotation_rads[2], axes="szyz"
            )


            # create message types for the pose target
            orientation = Quaternion(
                quaternion[0], quaternion[1], quaternion[2], quaternion[3]
            )
            # this is the position of the palm and the index retrieves the x,y,z values from the 
            # homogeneous transformation matrix
            position = Point(
                homogeneous_mat[0, 3], homogeneous_mat[1, 3], homogeneous_mat[2, 3] # type: ignore
            )
            # Set the target pose message
            pose_target = Pose(position, orientation)
            self.add_pose_arrow(position, rotation_rads[2])

            # print the pose target for debugging purposes
            #print("pose_target: ", pose_target)

            input("[PROGRESS] Pose Target Set, Press Enter to Continue...")
            self.arm.set_pose_target(pose_target)

        
        else:
            # Set the target orientation initalize position and orientation messages
            rotation = [
                math.radians(rotation[0]),
                math.radians(rotation[1]),
                math.radians(rotation[2]),
            ]
            orientation = quaternion_from_euler(
                rotation[0], rotation[1], rotation[2], axes="rzyz"
            )
            orientation = Quaternion(
                orientation[0], orientation[1], orientation[2], orientation[3]
            )
            position = Point(translation[0], translation[1], translation[2])
            # Set the target pose message
            pose_target = Pose(position, orientation)
            self.arm.set_pose_target(pose_target, end_effector_link="panda_link8")

        self.arm.set_goal_tolerance(0.001)
        self.arm.go(wait=True)
        self.arm.stop()
        # Clear the target
        self.arm.clear_pose_targets()

    def close_gripper(self) -> None:
        """Close the gripper"""
        print(
            f"[INFO] Current End Effector Joint Values (Closed): {[round(item, 3) for item in self.end_effector.get_current_joint_values()]}"
        )
        # using the CLOSE_GRIPPER variable set the panda_finger_joint1 and panda_finger_joint2 to the desired value
        self.end_effector.set_joint_value_target(CLOSE_GRIPPER)
        self.end_effector.go(wait=True)

    def open_gripper(self) -> None:
        """Open the gripper"""
        print(
            f"[INFO] Current End Effector Joint Values (Open): {[round(item, 3) for item in self.end_effector.get_current_joint_values()]}"
        )
        # using the OPEN_GRIPPER variable set the panda_finger_joint1 and panda_finger_joint2 to the desired value
        self.end_effector.set_joint_value_target(OPEN_GRIPPER)
        self.end_effector.go(wait=True)

    # attach the rod collision object to grasping group
    def attach_rod(self) -> None:
        """Attach an the rod to the robot"""
        grasping_group = "panda_hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, "rod", touch_links=touch_links)

    ## FLOW METHODS
    def pick_rod(self) -> None:
        rod_position = self.get_rod_position()
        # print the rod position for debugging purposes
        print("rod_position: ", rod_position)

        self.go_to_pose_target(
            [
                round(rod_position[0], 2),
                round(rod_position[1], 2),
                round(rod_position[2], 2),
            ],
            [45, 90, 45],
            relative=False,
        )
        self.close_gripper()
        # attach the rod to the end effector
        self.attach_rod()
        self.remove_pose_arrow()

    def place_rod(self) -> None:
        place_point_position = [
            box2["x_pos"],
            box2["y_pos"],
            rod_height / 2 + box2["z_height"],
        ]
        self.go_to_pose_target(place_point_position, [45, 90, 90], relative=False)
        self.remove_pose_arrow()
        self.open_gripper()
        # detach rod from end effector
        self.scene.remove_attached_object(self.eef_link, name="rod")

    ## UTILITY METHODS
    def get_rod_position(self) -> list:
        """Get the rod position"""
        rod_position = self.scene.get_object_poses(["rod"])["rod"].position
        # convert geometry_msgs.msg.Point to list
        return [rod_position.x, rod_position.y, rod_position.z]

    ## MARKER METHODS
    def add_pose_arrow(self, desired_position: Point, z_rotation: float) -> None:
        """Publish a marker at the desired pose"""
        marker = visualization_msgs.msg.Marker()
        marker.ns = "arrow"
        marker.id = 0
        marker.header.frame_id = "panda_link0"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale = Vector3(end_effector_palm_length, 0.02, 0.02)
        marker.color = std_msgs.msg.ColorRGBA(1.0, 0.0, 1.0, 1.0)

        quaternion = quaternion_from_euler(0, 0, z_rotation, axes="szyz")
        desired_pose = Pose(
            desired_position,
            Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
        )

        # print the desired pose for debugging purposes
        print("desired_pose: ", desired_pose)

        marker.pose = desired_pose
        # Publish the marker
        self.pose_point_pub.publish(marker)

    def remove_pose_arrow(self) -> None:
        """Remove the marker from the scene"""
        marker = visualization_msgs.msg.Marker()
        marker.ns = "arrow"
        marker.id = 0
        marker.action = marker.DELETE
        self.pose_point_pub.publish(marker)


def main(pick_and_place):
    try:
        # retrieve critical parameters and set up the scene
        home_joint_pos = pick_and_place.arm.get_current_joint_values()
        print(
            f"[INFO] Setting up Scene - Home Joint Values: {[round(item, 3)for item in home_joint_pos]}"
        )
        pick_and_place.create_collision_scene()
        input(
            "[PROGRESS] Scene Set Up and Initial Pose Identified, Press any key to Pick up the Rod"
        )
        # send the panda arm to the desired pose
        pick_and_place.pick_rod()
        input("[PROGRESS] Rod Picked, Press any key to place\n")
        pick_and_place.place_rod()
        input("[PROGRESS] Rod Placed, Press any key to move to home position\n")
        pick_and_place.go_to_joint_state(home_joint_pos)
        input(
            "[PROGRESS] Home Position Reached, Press any key to open the gripper and remove the objects from the scene"
        )
        pick_and_place.clean_scene()

    except rospy.ROSInterruptException:
        pick_and_place.clean_scene()
        # exit the program
        return


if __name__ == "__main__":
    # initialize the pick and place node
    pick_and_place = PickAndPlace()
    # run the main function
    main(pick_and_place)
