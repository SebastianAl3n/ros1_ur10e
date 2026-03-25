#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

class PickAndPlace(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)  # initialize moveit_commander and rospy
        rospy.init_node("pick_and_place_node", anonymous=True)

        self.robot = moveit_commander.RobotCommander()  # instantiate a RobotCommander object
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("robot_planning")  # instantiate a MoveGroupCommander object
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper_group")
        rospy.sleep(2)  # wait for the planning scene to be ready

    def adding_obstacles(self):
        '''Add the obstacles for pick and plase task to the planning scene'''
            # Cube definitions: (name, x, y, z, size)
        cubes = [("large_cube",0.0, -1.0, 0.755, 0.1),
                 ("med_cube", -0.5, -1.0, 0.755, 0.07),
                 ("small_cube", 0.5, -1.0, 0.755, 0.05)]