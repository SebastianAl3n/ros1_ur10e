#!/usr/bin/env python3
import sys
import moveit_commander
import rospy
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class RobotControl(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group = moveit_commander.MoveGroupCommander("robot_planning")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

        self.arm_group.set_goal_position_tolerance(0.005)
        self.arm_group.set_goal_orientation_tolerance(0.1)
        self.arm_group.set_planning_time(10.0)

        rospy.loginfo("Connecting to Gazebo Link Attacher...")
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
        self.attach_srv.wait_for_service()

        rospy.loginfo("Link Attacher service found!")
        
        rospy.sleep(2)  

    def gazebo_attach(self):       #attaching the object in Gazebo 
        req = AttachRequest()
        req.model_name_1 = "robot"
        req.link_name_1 = "wrist_3_link" 
        req.model_name_2 = "my_cube"
        req.link_name_2 = "cube_link"
        
        res = self.attach_srv.call(req)
        return res

    def gazebo_detach(self):
        req = AttachRequest()
        req.model_name_1 = "robot"
        req.link_name_1 = "wrist_3_link"
        req.model_name_2 = "my_cube"
        req.link_name_2 = "cube_link"
        
        res = self.detach_srv.call(req)
        return res

    def object_collision(self):      #adding the object in moveit planning scene
        cube_pose = geometry_msgs.msg.PoseStamped()
        cube_pose.header.frame_id = "world"
        cube_pose.pose.position.x = 0.0
        cube_pose.pose.position.y = -1.0
        cube_pose.pose.position.z = 0.97
        cube_pose.pose.orientation.w = 1.0

        self.scene.add_box("object", cube_pose, size=(0.06, 0.06, 0.06))


    def attach_object(self):
        gripper_links = [
            "onrobot_rg2_base_link", "left_outer_knuckle", "right_outer_knuckle",
            "left_inner_knuckle", "right_inner_knuckle",
            "left_inner_finger", "right_inner_finger"
        ]
        self.scene.attach_box("onrobot_rg2_base_link", "object", touch_links=gripper_links)  # This tells MoveIt the object moves with the robot now
        self.gazebo_attach()
        rospy.loginfo("object attached to gripper.")

    def detach_object(self):
        self.gazebo_detach()
        
        self.scene.remove_attached_object("onrobot_rg2_base_link", "object")
        
        self.scene.remove_world_object("object") 
        
        rospy.loginfo("object detached and released.")

    def go_to_joints(self, joint_goal): # path planning using joint angles (less precise, but often more reliable)
        self.arm_group.set_joint_value_target(joint_goal)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        return success

    def pose_in_quaternion( self, x, y, z, roll, pitch, yaw):
        pose_target = geometry_msgs.msg.Pose()
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        return pose_target

    def go_to_pose(self, x, y, z, roll, pitch, yaw): # path planning using euler angles
 
        pose_target = self.pose_in_quaternion(x, y, z, roll, pitch, yaw)
        self.arm_group.set_pose_target(pose_target)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def move_linear(self, dx, dy, dz): #Cartesian path planning for straight line movements (e.g., for precise grasping or placing)
        waypoints = []
        
        wpose = self.arm_group.get_current_pose().pose
        
        wpose.position.x += dx
        wpose.position.y += dy
        wpose.position.z += dz
        waypoints.append(geometry_msgs.msg.Pose(wpose.position, wpose.orientation))

        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, False)
        
        if fraction > 0.9: # If MoveIt found a path for at least 90% of the distance
            self.arm_group.execute(plan, wait=True)
            return True
        else:
            rospy.logerr("Cartesian path planning failed!")
            return False
    
    def go_to_pose_multi_planner(self, x, y, z, roll, pitch, yaw):
        pose_target = geometry_msgs.msg.Pose()
        q = quaternion_from_euler(roll, pitch, yaw)

        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        planners = [
            ("RRTConnectkConfigDefault", 5.0),
            ("PTP", 3.0)   # Pilz planner
        ]

        for planner_id, planning_time in planners:
            rospy.loginfo(f"Trying planner: {planner_id}")
            
            self.arm_group.set_planner_id(planner_id)
            self.arm_group.set_planning_time(planning_time)
            self.arm_group.set_pose_target(pose_target)

            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()

            if success:
                rospy.loginfo(f"Success with planner: {planner_id}")
                return True
            else:
                rospy.logwarn(f"Planner {planner_id} failed, trying next...")

        rospy.logerr("All planners failed!")
        return False
    
    def go_to_position_only(self, x, y, z):
        # This tells MoveIt! to ONLY care about the XYZ
        self.arm_group.set_position_target([x, y, z])
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success
    
    def go_to_pose_pilz(self,x,y,z,roll,pitch,yaw):
        rospy.loginfo("Planer changed to Pilz LIN")
        self.arm_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        self.arm_group.set_planner_id("LIN")
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_max_acceleration_scaling_factor(0.1)
        pose_target = self.pose_in_quaternion(x,y,z,roll,pitch,yaw)

        self.arm_group.set_pose_target(pose_target)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success
        

    def gripper_control(self, value): #-0.4 to open fully
        self.gripper_group.set_joint_value_target([value])
        self.gripper_group.go(wait=True)
