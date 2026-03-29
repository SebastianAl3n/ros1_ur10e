#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

class PickAndPlace(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_and_place_node", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Ensure these match your SRDF group names exactly
        self.arm_group = moveit_commander.MoveGroupCommander("robot_planning")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

        # Set tolerances to be a bit more "forgiving"
        self.arm_group.set_goal_position_tolerance(0.005)
        self.arm_group.set_goal_orientation_tolerance(0.01)
        self.arm_group.set_planning_time(6.0)
        
        rospy.sleep(2)

    def lebkuchen_collision(self):
        # Adding the box to the mental map
        kuchen_pose = geometry_msgs.msg.PoseStamped()
        kuchen_pose.header.frame_id = "world"
        kuchen_pose.pose.position.x = 0.277
        kuchen_pose.pose.position.y = -0.774
        # Table (0.8) + half cake height (approx 0.04)
        kuchen_pose.pose.position.z = 0.858
        kuchen_pose.pose.orientation.w = 1.0

        self.scene.add_box("lebkuchen", kuchen_pose, size=(0.1, 0.12, 0.085))
        rospy.loginfo("Lebkuchen added to planning scene")

    def go_to_joints(self, joint_goal):
        """Moves robot using raw joint angles (Reliable)"""
        self.arm_group.set_joint_value_target(joint_goal)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        return success

    def go_to_pose(self, x, y, z, roll, pitch, yaw):
        """Moves robot using XYZ and Euler angles (Precise)"""
        pose_target = geometry_msgs.msg.Pose()
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        
        self.arm_group.set_pose_target(pose_target)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def gripper_control(self, value):
        """0.0 for open, approx 0.4 for closed"""
        self.gripper_group.set_joint_value_target([value])
        self.gripper_group.go(wait=True)

if __name__ == "__main__":
    try:
        pnp = PickAndPlace()
        
        # 1. Setup Scene
        pnp.lebkuchen_collision()
        rospy.sleep(1)

        # 2. Move to READY position (using the Joint Goal that worked)
        target_x = 0.271
        target_y = -0.666
        target_z = 0.857  
        roll, pitch, yaw = -1.974, 1.554, 2.695

        rospy.loginfo("Moving to front of the Lebkuchen...")
        #pnp.go_to_pose(target_x, target_y +0.15, target_z, roll, pitch, yaw)

        #ready_joints = [-3.588, -1.308, -1.646, -3.346, -0.402, -3.142]
        ready_joints_home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        #ready_joints = [-3.588, -1.308, -1.646,-3.346, -1.57, 0.0]
        ready_joints = [-3.361, -1.208, -1.711,-3.398, -0.171, 0.019] # in front of the cake
        pnp.go_to_joints(ready_joints)

        rospy.loginfo("Positioned in front of the Lebkuchen, gripper opening...")
        pnp.gripper_control(-0.422)  # Close gripper to pick up the cake
    
    except rospy.ROSInterruptException:
        pass