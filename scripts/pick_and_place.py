import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler, euler_matrix,translation_matrix
import shape_msgs.msg
import sys
import math
import visualization_msgs.msg 
import std_msgs.msg
import numpy as np

# Constants for opening and closing the gripper
OPEN_GRIPPER = [0.035, 0.035]
CLOSE_GRIPPER = [0.011, 0.011]
end_effector_palm_length = 0.058 * 1.8 # 1.4 is padding
# box1 parameters
box1_height = 0.2
box1_xposition = 0.6
box1_yposition = 0.2
# box2 parameters
box2_height = 0.1
box2_xposition = 0
box2_yposition = 0.6
# rod parameters
rod_height = 0.2

class PickAndPlace(object):
    """Pick and place a box using moveit_commander"""

    def __init__(self):
        super(PickAndPlace, self).__init__()

        # Initialize moveit_commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('my_interfaces', anonymous=True)

        self.pose_point_pub = rospy.Publisher('pose_point', visualization_msgs.msg.Marker, queue_size=10)
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


    def create_collision_scene(self) -> None:
        """Create a collision scene"""
        # floor limit 
        floor = self.create_collision_object("floor", [2.5, 2.5, 0.01], [0, 0, -0.01], 0)
        self.scene.add_object(floor)
        box1 = self.create_collision_object("box1", [0.2, 0.4, box1_height], [box1_xposition, box1_yposition, box1_height/2], 0)
        self.scene.add_object(box1)
        box2 = self.create_collision_object("box2", [0.3, 0.2, box2_height], [box2_xposition, box2_yposition, box2_height/2], 0)
        self.scene.add_object(box2)
        rod = self.create_collision_object("rod", [0.02, 0.02, rod_height], [box1_xposition, box1_yposition, rod_height/2 + box1_height], 45)
        self.scene.add_object(rod)


    def get_joint_values(self):
        """Get the starting joint values"""
        return self.arm.get_current_joint_values()


    def go_to_joint_state(self, joint_goal) -> None:
        """Go to a specified joint state"""
        self.arm.go(joint_goal, wait=True)
        self.arm.stop()
    

    def close_gripper(self) -> None:
        """Close the gripper"""
        self.end_effector.set_named_target("close")
        self.end_effector.go()


    def go_to_pose_target(self, translation:list = None, rotation:list = None, relative:bool = False) -> None:
        """Go to a specified pose, the pose is selected by the user as the desired location
        of the palm of the hand. The orientation is specified by the user as a list of Euler angles
        and the rotation in which they are applied (static or relative needs to be specified in the 
        axis=<order> flag of the euler tf functions)."""

        if relative is False:
            # Set the target orientation initalize position and orientation messages
            rotation_rads = [math.radians(rotation[0]), math.radians(rotation[1]), math.radians(rotation[2])]
            homogeneous_mat_arm = euler_matrix(rotation_rads[0], rotation_rads[1], rotation_rads[2], axes='szyz')
            # add the translation to the homogeneous transformation matrix 
            homogeneous_mat_arm[:3, 3] = translation[:3]     
            # create a homogeneous transformation matrix for the end effector with no rotation
            homogeneous_trans_end_effector = translation_matrix([0, 0, end_effector_palm_length])
            # multiply the homogeneous transformation matrix of the arm by the inverse of the homogeneous transformation matrix of the end effector
            homogeneous_mat = np.dot(homogeneous_mat_arm, np.linalg.inv(homogeneous_trans_end_effector))
            # quaternion_from_euler using input rotation values 
            quaternion = quaternion_from_euler(rotation_rads[0], rotation_rads[1], rotation_rads[2], axes='szyz')           
            # create message types for the pose target
            orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            position = Point(homogeneous_mat[0,3], homogeneous_mat[1,3], homogeneous_mat[2,3])
            # Set the target pose message
            pose_target = Pose(position, orientation)
            self.add_pose_arrow(position, rotation_rads[2])
            input("Pose Target Set, Press Enter to Continue...")
            self.arm.set_pose_target(pose_target)


        # Needs Debugging - Path Planning Error
        else:
            # Set the target orientation initalize position and orientation messages
            rotation = [math.radians(rotation[0]), math.radians(rotation[1]), math.radians(rotation[2])]
            orientation = quaternion_from_euler(rotation[0], rotation[1], rotation[2], axes='rzyz')
            orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
            position = Point(translation[0], translation[1], translation[2])
            # Set the target pose message
            pose_target = Pose(position, orientation)
            self.arm.set_pose_target(pose_target, end_effector_link="panda_link8")
            
        self.arm.set_goal_tolerance(0.001)
        self.arm.go(wait=True)
        self.arm.stop()
        # Clear the target
        self.arm.clear_pose_targets()   

        
    def create_collision_object(self, id, dimensions:list, position:list, rotation_z:int) -> moveit_msgs.msg.CollisionObject:
        """Create collision objects and return it."""
        object = moveit_msgs.msg.CollisionObject()
        object.header.frame_id = self.planning_frame # "panda_link0" satisfies the std_msgs.msg.Header requirements
        object.id = id
    
        solid = shape_msgs.msg.SolidPrimitive()
        solid.type = solid.BOX
        solid.dimensions = dimensions
        object.primitives = [solid]

        # rotation about the z axis 
        rotation = [0, 0, math.radians(rotation_z)]
        quaternion = quaternion_from_euler(rotation[0], rotation[1], rotation[2], axes='szyz')

        # Position messages initailized and assigned
        object_point = Point(position[0], position[1], position[2])
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        object_pose = Pose(object_point, orientation)

        object.primitive_poses = [object_pose]
        object.operation = object.ADD
        return object

    # attach the rod collision object to grasping group
    def attach_rod(self) -> None:
        """Attach an object to the robot"""
        grasping_group = "panda_hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, "rod", touch_links=touch_links)


    def get_rod_position(self) -> None:
        """Get the rod position"""
        # returns a list of the rod position
        rod_position = self.scene.get_object_poses(["rod"])["rod"].position
        # convert geometry_msgs.msg.Point to list
        return [rod_position.x, rod_position.y, rod_position.z]


    def clean_scene(self) -> None:
        """Clean the scene"""
        self.scene.remove_world_object("floor")
        self.scene.remove_world_object("box1")
        self.scene.remove_world_object("box2")
        self.scene.remove_world_object("rod")


    def close_gripper(self) -> None:
        """Close the gripper"""
        # using the CLOSE_GRIPPER variable set the panda_finger_joint1 and panda_finger_joint2 to the desired value
        print(f"Current End Effector Joint Values (Closed): {self.end_effector.get_current_joint_values()}")
        self.end_effector.set_joint_value_target(CLOSE_GRIPPER)
        self.end_effector.go(wait=True)
        

    def open_gripper(self) -> None:
        """Open the gripper"""
        # using the OPEN_GRIPPER variable set the panda_finger_joint1 and panda_finger_joint2 to the desired value
        print(f"Current End Effector Joint Values (Open): {self.end_effector.get_current_joint_values()}")
        self.end_effector.set_joint_value_target(OPEN_GRIPPER)
        self.end_effector.go(wait=True)


    def add_pose_arrow(self, desired_position:Point, z_rotation:float) -> None:
        """Publish a marker at the desired pose"""    

        # Set up the marker message- edit the marker to display an arrow given orientation and positon
        marker = visualization_msgs.msg.Marker()
        marker.ns = "arrow"
        marker.id = 0
        marker.header.frame_id = 'panda_link0'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale = Vector3(end_effector_palm_length, 0.02, 0.02)
        marker.color = std_msgs.msg.ColorRGBA(1.0, 0.0, 1.0, 1.0)

        quaternion = quaternion_from_euler(0, 0, z_rotation, axes='szyz')
        desired_pose = Pose(desired_position, Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

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

    def move_home(self, home_joint_pos) -> None:
        """Move the robot to the home position"""
        self.go_to_joint_state(home_joint_pos)


    def pick_rod(self) -> None:
        rod_position = self.get_rod_position()
        self.go_to_pose_target([round(rod_position[0],2), round(rod_position[1],2), round(rod_position[2],2)], [45, 90, 45], relative=False)
        input("[PROGRESS] Pose Reached, Press any key to close the gripper")
        self.close_gripper()
        # attach the rod to the end effector
        self.attach_rod()
        self.remove_pose_arrow()
    
    def place_rod(self) -> None:
        place_point_position = [box2_xposition, box2_yposition, rod_height/2 + box2_height]
        self.go_to_pose_target(place_point_position, [45, 90, 90], relative=False)
        self.remove_pose_arrow()
        self.open_gripper()
        # detach rod from end effector
        self.scene.remove_attached_object(self.eef_link, name="rod")



def main(empty=False):
    try:
        # initialize the node
        pick_and_place = PickAndPlace()
        # retrieve critical parameters and set up the scene
        home_joint_pos = pick_and_place.get_joint_values()
        print(f"Joint Values: {home_joint_pos}")

        if empty is False:
            print("[INFO] Setting up scene with Collision Objects")
            pick_and_place.create_collision_scene()
            input("[PROGRESS] Scene Set Up and Initial Pose Identified, Press any key to Pick up the Rod")
            # send the panda arm to the desired pose
            pick_and_place.pick_rod()
            input("[PROGRESS] Rod Picked, Press any key to place")
            pick_and_place.place_rod()
            input("[PROGRESS] Rod Placed, Press any key to move to home position")
            pick_and_place.move_home(home_joint_pos)
            input("[PROGRESS] Home Position Reached, Press any key to open the gripper and remove the objects from the scene")
            pick_and_place.clean_scene()

        else:
            print("[INFO] Setting up Empty Scene")
            '''For testing purposes, you can set up the scene without any collision objects'''
            # pick_and_place.add_pose_arrow([0.59, 0.0, 0.45], [0, 90, 45])
            # input("[PROGRESS] Scene Set Up and Initial Pose Identified, Press any key to move to desired pose")
            # # send the panda arm to the desired pose
            # pick_and_place.go_to_pose_target([0.59, 0.0, 0.45], [0, 90, 45])
            # input("[PROGRESS] Pose Reached, Press any key to close the gripper/move to home position")
            # #pick_and_place.close_gripper()
            #input("[PROGRESS] Gripper Closed, Press any key to move to home position")
            
            pick_and_place.go_to_pose_target([0.6, 0.2, 0.6], [45, 90, 45], relative=False) 
            input("[PROGRESS] Static Pose Reached, Press any key to continue")
            
            
            # try a relative move
            #pick_and_place.go_to_pose_target([0.0, 0.0, 0.05], [0, 0, 0], relative=True)
            #input("[PROGRESS] Relative Pose Reached, Press any key to continue")

            pick_and_place.move_home(home_joint_pos)
            pick_and_place.remove_pose_arrow()

    
    except rospy.ROSInterruptException:
        pass
        
if __name__ == "__main__":
    main(empty=False)
        



