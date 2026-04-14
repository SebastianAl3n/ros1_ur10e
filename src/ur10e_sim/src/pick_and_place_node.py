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
    start_joints = [-1.954,-1.920,0.949,-5.330,1.275,-3.181]
    robot.go_to_joints(start_joints)
   


    target_pose = vision.extract_pose()

    if target_pose:
        world_x = target_pose.x
        world_y = target_pose.y
        world_z = target_pose.z
        rospy.loginfo(f"Success..found object at World Frame X={world_x:.3f}m, Y={world_y:.3f}m, Z={world_z:.3f}m")

        rospy.loginfo(f"Moving to pickup pose")
        robot.go_to_pose(world_x-0.02,world_y+0.1,world_z-0.05,-1.351, 1.533, -2.935)
        rospy.loginfo("gripper opening")
        robot.gripper_control(-0.39)
        robot.move_linear(0,-0.06,0)
        robot.gripper_control(-0.19)

    else:
        rospy.loginfo(f"No Pose was found")
    

if __name__ == "__main__":
    try:
      main()
        
    
    except rospy.ROSInterruptException:
        pass