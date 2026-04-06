#!/usr/bin/env python3
import rospy
from robot_utils import RobotControl
   
if __name__ == "__main__":
    try:
        rospy.init_node("pick_and_place_node", anonymous=True)
        robot = RobotControl()
        
        robot.lebkuchen_collision()
        rospy.sleep(1)

        # 2. Move to READY position (using the Joint Goal that worked)
        target_x = 0.274
        target_y = -0.579
        target_z = 0.858  
        roll, pitch, yaw = 1.953, -1.57, -0.431

        rospy.loginfo("Moving to front of the Lebkuchen...")
        robot.gripper_control(-0.55)  
        rospy.loginfo("moving to grasping pose...")
        robot.go_to_pose(target_x, target_y+0.05, target_z-0.005, roll, pitch, yaw)
        rospy.sleep(0.1)
        robot.move_linear(-0.005, -0.14, 0.0)  # Move straight down to the cake
        rospy.loginfo("Grasp pose reached, closing gripper...")
        robot.gripper_control(-0.4)
        rospy.sleep(0.1)
        rospy.loginfo("Lifting the Lebkuchen...")

        rospy.loginfo("Activating Link Attacher...")
        robot.attach_lebkuchen()
        rospy.loginfo("Lifting the Lebkuchen...")
        robot.go_to_pose(target_x, target_y, target_z + 0.2, roll, pitch, yaw)
        rospy.loginfo("Pick successful!")
        rospy.sleep(0.1)
        place_joint_01 = [1.185, -1.720, 1.186, 0.536, -5.042, 0.0]
        robot.go_to_joints(place_joint_01)
        place_joint_02 = [1.244, -1.521, 1.039, 0.825, -4.968, 0.0]
        robot.go_to_joints(place_joint_02)
        rospy.sleep(0.1)
        rospy.loginfo("place pose reached")
        robot.detach_lebkuchen()
        rospy.loginfo("Lebkuchen placed!")
        place_joint_03 = [1.222, -1.606, 1.033, 0.509, -4.969, 0.0]
        robot.go_to_joints(place_joint_03)
        
    
    except rospy.ROSInterruptException:
        pass