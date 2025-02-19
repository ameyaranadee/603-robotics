#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

x = 0.0
y = 0.0
yaw = 0.0

def poseCallback(pose_message):
    global x, y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

def go_to_goal(x_goal, y_goal):
    global x, y, yaw
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    velocity_message = Twist()
    K_linear = 1.0
    K_angular = 6.0

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        distance = math.sqrt((x_goal - x)**2 + (y_goal - y)**2)
        if distance < 0.01:
            break

        # Proportional control
        linear_speed = K_linear * distance
        desired_angle_goal = math.atan2(y_goal - y, x_goal - x)
        angle_error = desired_angle_goal - yaw
        # Keep angle_error in range [-pi, pi]
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        angular_speed = K_angular * angle_error

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

        rate.sleep()

    # Stop the turtle
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)

def main():
    rospy.init_node('turtlesim_motion_pose', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
    time.sleep(2)

    go_to_goal(3.0, 9.0)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
