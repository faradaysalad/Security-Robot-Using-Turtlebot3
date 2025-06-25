#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
import math

class DoorPatrolAndYawChecker:
    def __init__(self):
        rospy.init_node('door_patrol_yaw_checker')

        # --- Publishers & Subscribers ---
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # --- Wait for Gazebo Service ---
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # --- State ---
        self.robot_position = (0.0, 0.0)
        self.checked_doors = {}
        self.goal_done = True
        self.detection_radius = 3  # Check if robot is close enough to a door

        # --- Door model names and their positions ---
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

        # --- Patrol goals ---
        self.door_coords = [
            (10.605456352233887, 3.0774009227752686),
            (5.659623622894287, 4.862634658813477),
            (1.6040650606155396, 5.165288925170898),
            (-2.955916404724121, 5.367058277130127),
            (-6.962615013122559, 4.7576069831848145),
            (-9.595630645751953, -1.1749746799468994),
            (2.403357744216919, -1.174976110458374),
            (2.866227149963379, 0.9969482421875),
            (6.035099506378174, -4.094611167907715)
        ]

        # Start checking and patrolling
        rospy.Timer(rospy.Duration(1.0), self.check_doors)
        rospy.sleep(2)
        self.patrol_doors()

    def odom_callback(self, msg):
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def result_callback(self, msg):
        if msg.status.status == 3:  # Goal reached
            self.goal_done = True
            rospy.loginfo("✅ Reached current goal.")

    def patrol_doors(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            for x, y in self.door_coords:
                self.goal_done = False
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.orientation.w = 1.0

                rospy.loginfo(f"📍 Moving to: ({x:.2f}, {y:.2f})")
                self.pub.publish(goal)

                while not rospy.is_shutdown() and not self.goal_done:
                    rate.sleep()

            rospy.loginfo("🔁 Finished one round of patrolling. Restarting...")

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

                    # Special doors have higher yaw threshold
                    if model_name in ["door_2", "door_2_clone", "door_2_clone_0"]:
                        state = "OPENED 🚨" if yaw > 2.3 else "CLOSED"
                    else:
                        state = "OPENED 🚨" if yaw > 1.0 else "CLOSED"

                    if prev_state != state:
                        print(f"🚪 {label} ({model_name}) is {state}")
                        self.checked_doors[model_name] = state

                except Exception as e:
                    rospy.logwarn(f"❌ Failed to get model state for {model_name}: {e}")

    def get_yaw_from_quaternion(self, z, w):
        return math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))


if __name__ == '__main__':
    try:
        DoorPatrolAndYawChecker()
    except rospy.ROSInterruptException:
        pass

