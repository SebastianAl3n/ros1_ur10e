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
    start_joints = [-3.579, -1.630, 0.06, 0.994, -0.676, -2.625]
    robot.go_to_joints(start_joints)


    target_pose = vision.extract_pose()

    if target_pose:
        world_x = target_pose.x
        world_y = target_pose.y
        world_z = target_pose.z
        rospy.loginfo(f"Success..found object at World Frame X={world_x:.3f}m, Y={world_y:.3f}m, Z={world_z:.3f}m")

        rospy.loginfo(f"Moving to pickup pose")
        robot.go_to_pose(world_x,world_y,world_z+0.19,3.045,1.501,1.477)
    
    else:
        rospy.loginfo(f"No Pose was found")
    

if __name__ == "__main__":
    try:
      main()
        
    
    except rospy.ROSInterruptException:
        pass