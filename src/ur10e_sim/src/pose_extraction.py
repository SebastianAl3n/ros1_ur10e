#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_utils import RobotControl
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.lower_green = np.array([35, 10, 10])
        self.upper_green = np.array([85, 255, 255])
        self.current_image = None
        self.current_depth = None

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)
        self.mask_pub = rospy.Publisher("/camera/green_mask", Image, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0)

    def callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_image = cv_img.copy() 
            
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Draw Bounding box for visualization
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 100:
                    x, y, w, h = cv2.boundingRect(largest)
                    cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(cv_img, (x + w//2, y + h//2), 5, (0, 0, 255), -1)

            self.annotated_image = cv_img
            cv2.imshow("Robot Live View", self.annotated_image)
            cv2.waitKey(1)
            

        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def depth_callback(self, data):
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(data,"32FC1")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def detect_green(self):
        if self.current_image is None:
            return None

        hsv_img = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, self.lower_green, self.upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        bbox = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 50: 
                x, y, w, h = cv2.boundingRect(largest_contour) #bounding box coordinate
                
                # Optional: Draw the box on the image for debugging
                cv2.rectangle(self.current_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                bbox = (x, y, w, h)

        #cv2.imshow("Robot View", self.current_image) 
        #cv2.imshow("Green Mask (What the robot thinks is green)", mask)
        #cv2.waitKey(1)
        
        return bbox
    
    def get_3d_pose(self, cx, cy):
        if self.current_depth is None:
            rospy.logwarn("No Depth info recieved")
            return None
        
        z_m = self.current_depth[cy,cx] # accessing the depth info at these pixels

        if np.isnan(z_m) or z_m <=0:
            rospy.logwarn("Invalid Depth Value at Centroid")

        #calculating the xyz of the centroid in the camera frame

        fx = 465.7
        fy = 465.7
        ox = 320 # center of 680 width
        oy = 240  # center of 480 height

        x_c = (cx - ox) * z_m / fx
        y_c = (cy - oy) * z_m / fy
        z_c = z_m

        return (x_c, y_c, z_c)

    def get_world_pose(self, cam_x, cam_y, cam_z):
        try:
            transform = self.tf_buffer.lookup_transform("world","camera_color_optical_frame",rospy.Time(0),rospy.Duration(1.0))

            p_cam = PointStamped()
            p_cam.header.frame_id = "camera_color_optical_frame"
            p_cam.point.x = cam_x
            p_cam.point.y = cam_y
            p_cam.point.z = cam_z

            p_world = tf2_geometry_msgs.do_transform_point(p_cam,transform)
            return p_world.point

        except Exception as e:
            rospy.logerr(f"transformation failed: {e}")
            return None
    
    def extract_pose(self):
        """function to run the whole vision pipeline in one call"""
        bbox = self.detect_green()
        if bbox:
            x, y, w, h = bbox
            cx, cy = x + (w // 2), y + (h // 2)

            pose_cam = self.get_3d_pose(cx, cy)
            if pose_cam:
                return self.get_world_pose(*pose_cam)
        
        return None




if __name__ == "__main__":
    try:
    # 1. Start the ROS node
        rospy.init_node("pose_extraction_node", anonymous=True)
        
        # 2. Initialize the tools (Start vision early so it's ready)
        vision = ColorDetector()
        robot = RobotControl()

        # 3. Move to the "Scanning" Pose
        rospy.loginfo("Moving to scanning position...")
        start_joints = [-3.579, -1.630, 0.06, 0.994, -0.676, -2.625]
        robot.go_to_joints(start_joints)
        
        # 4. Wait for the arm to stop shaking and the camera to focus
        rospy.sleep(2.0)

        # 5. Call the detection method and capture the result
        rospy.loginfo("Scanning for green objects...")
        bbox = vision.detect_green()

        # 6. Act based on the result
        if bbox:
            x, y, w, h = bbox
            center_x = x + (w // 2)
            center_y = y + (h // 2)
            rospy.loginfo(f"SUCCESS: Found green object at Center ({center_x}, {center_y})")

            pose_cam = vision.get_3d_pose(center_x,center_y)

            if pose_cam:
                x_c, y_c, z_c = pose_cam
                rospy.loginfo(f"Success..found object at Camera Frame X={x_c:.3f}m, Y={y_c:.3f}m, Z={z_c:.3f}m")

                pose_world = vision.get_world_pose(x_c, y_c, z_c)

                if pose_world:
                    world_x = pose_world.x
                    world_y = pose_world.y
                    world_z = pose_world.z
                    rospy.loginfo(f"Success..found object at World Frame X={world_x:.3f}m, Y={world_y:.3f}m, Z={world_z:.3f}m")

                else:
                    rospy.logwarn("Transformation was not successfull")

            else:
                rospy.logwarn("Target found in 2D but 3D Pose extraction failed")
            
            # This is where you'd add: robot.go_to_pose(...) to grab it!
        else:
            rospy.logwarn("FAILED: No green object detected. Check lighting or HSV values.")

        # 7. Keep windows alive so you can inspect the Mask
        if vision.current_image is not None:
            rospy.loginfo("Displaying debug windows. Press any key in the window to exit.")
            cv2.waitKey(0) 
            cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass