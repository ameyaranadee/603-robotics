#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi

class TurtleController:
    def __init__(self):
        # Initialize node
        rospy.init_node('P1D2_turtle_controller', anonymous=True)
        
        # Publisher to control the turtle's movement
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to get the turtle's current position
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        
        # Current pose of the turtle
        self.pose = Pose()
        
        # Control rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Waypoints for the M shape (tuned for turtlesim's 11x11 coordinate system)
        # The M shape starts from bottom left, goes up, diagonally down to middle, 
        # diagonally up to top right, then down
        self.waypoints = [
            (2.0, 2.0),     # Starting point (bottom left)
            (2.0, 9.0),     # Top left
            (5.5, 5.5),     # Middle bottom
            (9.0, 9.0),     # Top right
            (9.0, 2.0),     # Bottom right
        ]
        
        # Control parameters
        self.distance_tolerance = 0.1
        self.angular_tolerance = 0.05
        
        # Maximum linear and angular velocities
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0
        
    def update_pose(self, data):
        """Callback function to update the turtle's current pose"""
        self.pose = data
        
    def euclidean_distance(self, goal_x, goal_y):
        """Calculate Euclidean distance between current position and goal"""
        return sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
    
    def linear_velocity(self, goal_x, goal_y, constant=1.5):
        """Calculate linear velocity using proportional control"""
        distance = self.euclidean_distance(goal_x, goal_y)
        velocity = constant * distance
        
        # Cap the velocity
        return min(velocity, self.max_linear_velocity)
    
    def steering_angle(self, goal_x, goal_y):
        """Calculate steering angle required to reach the goal"""
        return atan2(goal_y - self.pose.y, goal_x - self.pose.x)
    
    def angular_velocity(self, goal_x, goal_y, constant=6.0):
        """Calculate angular velocity using proportional control"""
        target_angle = self.steering_angle(goal_x, goal_y)
        current_angle = self.pose.theta
        
        # Calculate the shortest angle difference
        angle_diff = target_angle - current_angle
        
        # Normalize the angle difference to be between -pi and pi
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi
        
        # Proportional control for angular velocity
        angular_velocity = constant * angle_diff
        
        # Cap the angular velocity
        return min(max(angular_velocity, -self.max_angular_velocity), self.max_angular_velocity)
    
    def move_to_point(self, goal_x, goal_y):
        """Move the turtle to the specified point with closed-loop control"""
        velocity_msg = Twist()
        
        # Continue until we reach the destination
        while self.euclidean_distance(goal_x, goal_y) >= self.distance_tolerance:
            # Check if ROS is still running
            if rospy.is_shutdown():
                break
                
            # Calculate velocities
            velocity_msg.linear.x = self.linear_velocity(goal_x, goal_y)
            velocity_msg.angular.z = self.angular_velocity(goal_x, goal_y)
            
            # Logging (optional)
            rospy.loginfo(f"Moving to ({goal_x:.2f}, {goal_y:.2f}), Distance: {self.euclidean_distance(goal_x, goal_y):.2f}")
            
            # Publish velocity commands
            self.velocity_publisher.publish(velocity_msg)
            
            # Sleep to maintain the control rate
            self.rate.sleep()
        
        # Stop the turtle once we reach the destination
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)
        
        rospy.loginfo(f"Reached point ({goal_x:.2f}, {goal_y:.2f})")
    
    def draw_m_shape(self):
        """Draw the M shape by moving through all waypoints"""
        rospy.loginfo("Starting to draw the M shape")
        
        # First, teleport to the starting position (if needed, this requires the reset service)
        # For this implementation, we'll assume the turtle starts at a reasonable position
        
        # Visit each waypoint in sequence
        for i, (x, y) in enumerate(self.waypoints):
            rospy.loginfo(f"Moving to waypoint {i+1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f})")
            self.move_to_point(x, y)
            
        rospy.loginfo("Finished drawing the M shape")

def main():
    try:
        # Create controller and draw M shape
        controller = TurtleController()
        
        # Small delay to ensure subscribers are ready
        rospy.sleep(1)
        
        # Draw the M shape
        controller.draw_m_shape()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

if __name__ == '__main__':
    main()