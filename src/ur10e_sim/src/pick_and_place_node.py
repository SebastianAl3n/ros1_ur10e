#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

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

        rospy.loginfo("Connecting to Gazebo Link Attacher...")
        # Make sure the names match exactly what you use in gazebo_attach()
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        
        # This prevents the script from running until the plugin is actually ready
        self.attach_srv.wait_for_service() 
        rospy.loginfo("Link Attacher service found!")
        
        rospy.sleep(2)

    def gazebo_attach(self):
        """Physical glue in Gazebo"""
        req = AttachRequest()
        req.model_name_1 = "robot"
        req.link_name_1 = "wrist_3_link" # The 'lumped' link we found
        req.model_name_2 = "lebkuchen"
        req.link_name_2 = "package_link"
        
        res = self.attach_srv.call(req)
        return res

    def gazebo_detach(self):
        """Physical release in Gazebo"""
        req = AttachRequest()
        req.model_name_1 = "robot"
        req.link_name_1 = "wrist_3_link"
        req.model_name_2 = "lebkuchen"
        req.link_name_2 = "package_link"
        
        res = self.detach_srv.call(req)
        return res

    def lebkuchen_collision(self):
        # Adding the box to the mental map
        kuchen_pose = geometry_msgs.msg.PoseStamped()
        kuchen_pose.header.frame_id = "world"
        kuchen_pose.pose.position.x = 0.277
        kuchen_pose.pose.position.y = -0.75
        # Table (0.8) + half cake height (approx 0.04)
        kuchen_pose.pose.position.z = 0.858
        kuchen_pose.pose.orientation.w = 1.0

        self.scene.add_box("lebkuchen", kuchen_pose, size=(0.08, 0.1, 0.075))


    def attach_lebkuchen(self):
        """Call this ONLY after the gripper is closed."""
        gripper_links = [
            "onrobot_rg2_base_link", "left_outer_knuckle", "right_outer_knuckle",
            "left_inner_knuckle", "right_inner_knuckle",
            "left_inner_finger", "right_inner_finger"
        ]
        # This tells MoveIt the box moves with the robot now
        self.scene.attach_box("onrobot_rg2_base_link", "lebkuchen", touch_links=gripper_links)
        self.gazebo_attach()
        rospy.loginfo("Lebkuchen attached to gripper.")

    def detach_lebkuchen(self):
        """Call this to release the object physically and mentally"""
        # 1. Break the physical glue in Gazebo
        self.gazebo_detach()
        
        # 2. Tell MoveIt the object is no longer moving with the arm
        self.scene.remove_attached_object("onrobot_rg2_base_link", "lebkuchen")
        
        # 3. Completely remove it from the planning scene if you don't want to hit it later
        # self.scene.remove_world_object("lebkuchen") 
        
        rospy.loginfo("Lebkuchen detached and released.")

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

    def move_linear(self, dx, dy, dz):
        """Moves the TCP in a straight line relative to current position"""
        waypoints = []
        
        # Get starting pose
        wpose = self.arm_group.get_current_pose().pose
        
        # Set target pose
        wpose.position.x += dx
        wpose.position.y += dy
        wpose.position.z += dz
        waypoints.append(geometry_msgs.msg.Pose(wpose.position, wpose.orientation))

        # Compute path (eef_step is 1cm, jump_threshold is 0.0)
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, False)
        
        if fraction > 0.9: # If MoveIt found a path for at least 90% of the distance
            self.arm_group.execute(plan, wait=True)
            return True
        else:
            rospy.logerr("Cartesian path planning failed!")
            return False

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
        target_x = 0.274
        target_y = -0.579
        target_z = 0.858  
        roll, pitch, yaw = 1.953, -1.57, -0.431

        rospy.loginfo("Moving to front of the Lebkuchen...")
        pnp.gripper_control(-0.55)  
        rospy.loginfo("moving to grasping pose...")
        pnp.go_to_pose(target_x, target_y+0.05, target_z-0.005, roll, pitch, yaw)
        rospy.sleep(0.1)
        pnp.move_linear(-0.005, -0.14, 0.0)  # Move straight down to the cake
        rospy.loginfo("Grasp pose reached, closing gripper...")
        pnp.gripper_control(-0.4)
        rospy.sleep(0.1)
        rospy.loginfo("Lifting the Lebkuchen...")

        rospy.loginfo("Activating Link Attacher...")
        pnp.attach_lebkuchen()
        rospy.loginfo("Lifting the Lebkuchen...")
        pnp.go_to_pose(target_x, target_y, target_z + 0.2, roll, pitch, yaw)
        rospy.loginfo("Pick successful!")
        rospy.sleep(0.1)
        place_joint_01 = [1.185, -1.720, 1.186, 0.536, -5.042, 0.0]
        pnp.go_to_joints(place_joint_01)
        place_joint_02 = [1.244, -1.521, 1.039, 0.825, -4.968, 0.0]
        pnp.go_to_joints(place_joint_02)
        rospy.sleep(0.1)
        rospy.loginfo("place pose reached")
        pnp.detach_lebkuchen()
        rospy.loginfo("Lebkuchen placed!")
        place_joint_03 = [1.222, -1.606, 1.033, 0.509, -4.969, 0.0]
        pnp.go_to_joints(place_joint_03)
        
    
    except rospy.ROSInterruptException:
        pass