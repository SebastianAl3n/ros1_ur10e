# ROS1_UR10e Workspace
A ROS1 workspace for controlling the ur10e and OnRobot attachments

# Starting the simulation
'''
$  cd ros1_ur10e/
$ source devel/setp.bash
$ roslaunch ur10e_sim full_robot.launch
$ roslaunch ur10e_sim moveit.launch
$ LIBGL_ALWAYS_SOFTWARE=1 roslaunch moveit_config moveit_rviz.launch
'''