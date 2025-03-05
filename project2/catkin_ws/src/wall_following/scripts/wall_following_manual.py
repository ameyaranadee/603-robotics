#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Q-table
        self.q_table = {
            'too_close': {'turn_left': 10, 'straight': 5, 'turn_right': 1},
            'desired': {'turn_left': 2, 'straight': 10, 'turn_right': 3},
            'too_far': {'turn_left': 1, 'straight': 5, 'turn_right': 10}
        }
        self.desired_distance = 0.75
        self.region_threshold = 0.25

    def get_state(self, ranges):
        # Extract right-side LiDAR data
        right_region = ranges[-60:] if len(ranges) == 360 else []
        valid_ranges = [r for r in right_region if not (r == float('inf') or r == 0.0)]
        if not valid_ranges:
            return 'desired'  # if no valid data
        
        min_distance = min(valid_ranges)
        if min_distance < self.desired_distance - self.region_threshold:
            return 'too_close'
        elif min_distance > self.desired_distance + self.region_threshold:
            return 'too_far'
        else:
            return 'desired'

    def select_action(self, state):
        return max(self.q_table[state], key=self.q_table[state].get)

    def scan_callback(self, msg):
        state = self.get_state(msg.ranges)
        action = self.select_action(state)
        
        twist = Twist()
        twist.linear.x = 0.2
        
        if action == 'turn_left':
            twist.angular.z = 0.5 
        elif action == 'turn_right':
            twist.angular.z = -0.5
        else:
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        WallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass