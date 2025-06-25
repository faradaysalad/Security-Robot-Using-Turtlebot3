#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
import math

class DoorYawChecker:
    def __init__(self):
        rospy.init_node('door_yaw_checker')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.robot_position = (0.0, 0.0)
        self.checked_doors = {}

        self.doors = {
            "door": {"position": (5.800480, -6.608840), "label": "Toilet 1 Door"},
            "door_0": {"position": (3.796760, -6.608150), "label": "Toilet 2 Door"},
            "door_1": {"position": (-10.281670, -5.619590), "label": "Office 1 Door"},
            "door_2": {"position": (-7.924440, 2.640270), "label": "Office 2 Door"},
            "door_2_clone": {"position": (1.169290, -0.379760), "label": "Meeting Room 1 Door"},
            "door_2_clone_0": {"position": (-2.383630, -2.383630), "label": "Meeting Room 2 Door"},
            "door_3": {"position": (-6.433410, 4.176660), "label": "Toilet 4 Door"},
            "door_4": {"position": (-4.308510, 4.171920), "label": "Toilet 3 Door"},
            "door_5": {"position": (1.631160, 4.199150), "label": "Room 1 Door"},
            "door_6": {"position": (4.697580, 4.208710), "label": "Room 2 Door"},
            "door_7": {"position": (10.651630, 2.203260), "label": "Meeting Room 3 Door"}
        }

        self.detection_radius = 1.5
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Timer(rospy.Duration(1.0), self.check_doors)
        rospy.spin()

    def odom_callback(self, msg):
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def check_doors(self, event):
        for model_name, data in self.doors.items():
            door_pos = data["position"]
            label = data["label"]
            dist = math.hypot(door_pos[0] - self.robot_position[0],
                              door_pos[1] - self.robot_position[1])

            if dist <= self.detection_radius:
                try:
                    result = self.get_model_state(model_name, "")
                    yaw = self.get_yaw_from_quaternion(result.pose.orientation.z, result.pose.orientation.w)
                    prev_state = self.checked_doors.get(model_name)

                    # Special yaw thresholds for some doors
                    if model_name in ["door_2", "door_2_clone", "door_2_clone_0"]:
                        state = "OPENED 🚨" if yaw > 2.0 else "CLOSED"
                    else:
                        state = "OPENED 🚨" if yaw > 1.0 else "CLOSED"

                    if prev_state != state:
                        print(f"🚪 {label} ({model_name}) is {state}")
                        self.checked_doors[model_name] = state

                except Exception as e:
                    rospy.logwarn(f"Failed to get model state for {model_name}: {e}")

    def get_yaw_from_quaternion(self, z, w):
        return math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))

if __name__ == '__main__':
    try:
        DoorYawChecker()
    except rospy.ROSInterruptException:
        pass

