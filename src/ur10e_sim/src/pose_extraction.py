#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_utils import RobotControl

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.lower_green = np.array([35, 10, 10])
        self.upper_green = np.array([85, 255, 255])
        self.current_image = None

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
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
                # Get the bounding box coordinates
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Optional: Draw the box on the image for debugging
                cv2.rectangle(self.current_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                bbox = (x, y, w, h)

        cv2.imshow("Robot View", self.current_image) 
        cv2.imshow("Green Mask (What the robot thinks is green)", mask)
        cv2.waitKey(1)
        
        return bbox



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