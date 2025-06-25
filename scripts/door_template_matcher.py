#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os

class DoorTemplateMatcher:
    def __init__(self):
        rospy.init_node('door_template_matcher', anonymous=True)

        self.bridge = CvBridge()
        self.seen_door = False
        self.min_distance = float('inf')

        # Load template from home directory
        template_path = os.path.expanduser("~/door_template.jpg")
        self.template = cv2.imread(template_path, 0)

        if self.template is None:
            rospy.logerr("❌ Failed to load door_template.jpg at ~/door_template.jpg")
            exit()

        self.template_w, self.template_h = self.template.shape[::-1]

        # Subscribers
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.loginfo("🚪 Door Template Matcher node started.")
        rospy.spin()

    def scan_callback(self, msg):
        ranges = [r for r in msg.ranges if not math.isinf(r)]
        if ranges:
            self.min_distance = min(ranges)

    def image_callback(self, msg):
        if self.seen_door:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Image conversion error: %s", str(e))
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(gray, self.template, cv2.TM_CCOEFF_NORMED)
        threshold = 0.6
        loc = np.where(res >= threshold)

        if len(loc[0]) > 0 and self.min_distance <= 3.0:
            rospy.loginfo("✅ Door detected within 3m using template match.")
            self.seen_door = True

if __name__ == '__main__':
    try:
        DoorTemplateMatcher()
    except rospy.ROSInterruptException:
        pass

