#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class PatrolNode:
    def __init__(self):
        rospy.init_node('door_patrol_node')
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)

        self.goal_done = True
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

        rospy.sleep(2)
        self.patrol_doors()

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

if __name__ == '__main__':
    try:
        PatrolNode()
    except rospy.ROSInterruptException:
        pass




