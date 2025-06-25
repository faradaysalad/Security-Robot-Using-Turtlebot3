#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import math

class DoorImageDetector:
    def __init__(self):
        rospy.init_node('door_image_detector', anonymous=True)

        self.bridge = CvBridge()
        self.seen_door = False
        self.min_distance = float('inf')

        # Subscribers
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.loginfo("Door Image Detector node started.")
        rospy.spin()

    def scan_callback(self, msg):
        # Get the closest object in front of the robot
        ranges = [r for r in msg.ranges if not math.isinf(r)]
        if ranges:
            self.min_distance = min(ranges)

    def image_callback(self, msg):
        if self.seen_door:
            return  # Already detected, ignore

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))
            return

        # Door detection logic (simple color/edge detection — adjust as needed)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Assume a door if any large rectangular contour found
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)
            if len(approx) == 4 and area > 5000 and self.min_distance <= 3.0:
                rospy.loginfo("🚪 Door detected within 3m!")
                self.seen_door = True
                break

if __name__ == '__main__':
    try:
        DoorImageDetector()
    except rospy.ROSInterruptException:
        pass

