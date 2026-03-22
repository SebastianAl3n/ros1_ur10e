#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

def move_to_home():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_home',anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator" #should change according to your MoveIt! setup
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.loginfo("Moving to home position...")

    move_group.set_named_target("home") #should change according to your MoveIt! setup

    plan = move_group.go(wait=True)