#!/usr/bin/env python3
import rospy
from robot_utils import RobotControl
from pose_extraction import ColorDetector
   

def main():
    rospy.init_node("pick_and_place_node",anonymous=True)
    robot = RobotControl()
    vision = ColorDetector()

    robot.object_collision()
    rospy.sleep(1)
    

    rospy.loginfo("Moving to scanning position...")
    vision.status = "SCANNING"
    start_joints = [-2.817,-2.234,1.450,0.602,0.282,-2.855]
    robot.go_to_joints(start_joints)
   


    target_pose = vision.extract_pose()

    if target_pose:
        world_x = target_pose.x
        world_y = target_pose.y
        world_z = target_pose.z
        rospy.loginfo(f"Success..found object at World Frame X={world_x:.3f}m, Y={world_y:.3f}m, Z={world_z:.3f}m")

        rospy.loginfo(f"Moving to pickup pose")
        robot.go_to_pose(world_x-0.02,world_y+0.1,world_z-0.03,-1.351, 1.533, -2.935)
        rospy.loginfo("gripper opening")
        robot.gripper_control(-0.39)
        robot.move_linear(0,-0.065,0)
        rospy.loginfo("gripper closing")
        robot.gripper_control(-0.2)
        robot.attach_object()
        rospy.loginfo("Object attached..moving to drop off pose")
        robot.move_linear(0,0,0.05)
        #drop_off_joints = [1.286,-1.504,0.962,0.586,1.270,-3.177]
        drop_off_joint_01 = [-4.953,-1.590,1.135,-2.386,-1.440,-0.224]
        drop_off_joint_02 = [-4.891,-1.249,0.758,-1.291,-1.552,-0.191]
        robot.go_to_joints(drop_off_joint_01)
        robot.go_to_joints(drop_off_joint_02)
        
        
        robot.gripper_control(-0.39)
        robot.detach_object()

        rospy.loginfo("Pick and Place task successful")
        robot.go_to_pose(-0.040, 0.791, 1.130,-3.081, 0.390, -1.543)

    else:
        rospy.loginfo(f"No Pose was found")
    

if __name__ == "__main__":
    try:
      main()
      #robot = RobotControl()
      #drop_off_joint_01 = [-4.953,-1.590,1.135,-1.733,-1.440,-0.224]
      #robot.go_to_joints(drop_off_joint_01)

    
    except rospy.ROSInterruptException:
        pass