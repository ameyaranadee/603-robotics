import rospy
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from math import pow, atan2, sqrt, pi

class TurtleController:
    def __init__(self, image_path):
        rospy.init_node('turtle_m_drawer', anonymous=True)
        
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        
        # Wait for teleport service
        rospy.wait_for_service('/turtle1/teleport_absolute')
        self.teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        
        # Wait for pen service
        rospy.wait_for_service('/turtle1/set_pen')
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        
        self.pose = Pose()
        self.rate = rospy.Rate(20)
        
        # Extract waypoints from the provided image
        self.waypoints = self.extract_waypoints_from_image(image_path)
        
        self.distance_tolerance = 0.05
        self.angular_tolerance = 0.05 
        self.max_linear_velocity = 2.0 
        self.max_angular_velocity = 3.0 

    def update_pose(self, data):
        """Callback to update the turtle's current pose."""
        self.pose = data

    def extract_waypoints_from_image(self, image_path):
        """Use OpenCV to find and approximate the largest contour, then map it into turtlesim coords."""
        image = cv2.imread(image_path)
        if image is None:
            rospy.logerr("Could not load image at: " + image_path)
            return []
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            rospy.logerr("No contours found in the image.")
            return []
        
        contour = max(contours, key=cv2.contourArea)
        
        epsilon = 0.005 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        points = approx.reshape(-1, 2)

        waypoints = self.transform_to_turtlesim(points, image.shape)

        # Rotate so that we start at the bottom-left corner, but preserve the original path order
        waypoints = self.rotate_to_bottom_left(waypoints)

        return waypoints

    def transform_to_turtlesim(self, points, image_shape):
        """Map image coordinates to turtlesim's coordinate system with proper scaling."""
        height, width = image_shape[:2]
        min_x = np.min(points[:, 0])
        max_x = np.max(points[:, 0])
        min_y = np.min(points[:, 1])
        max_y = np.max(points[:, 1])
        
        if max_x == min_x or max_y == min_y:
            rospy.logerr("Contour is degenerate (zero width/height).")
            return []
        
        margin = 1.0
        turtlesim_min = margin
        turtlesim_max = 11.0 - margin
        
        transformed = []
        for (x, y) in points:
            tx = (x - min_x) / float(max_x - min_x) * (turtlesim_max - turtlesim_min) + turtlesim_min
            ty = turtlesim_max - (y - min_y) / float(max_y - min_y) * (turtlesim_max - turtlesim_min)
            transformed.append((tx, ty))
        return transformed

    def rotate_to_bottom_left(self, waypoints):
        """
        Keep the path order from approxPolyDP but rotate the list so
        that the bottom-left corner is the first point.
        """
        if not waypoints:
            return []
        
        bottom_left_idx = min(range(len(waypoints)),
                              key=lambda i: (waypoints[i][1], waypoints[i][0]))
        
        waypoints = waypoints[bottom_left_idx:] + waypoints[:bottom_left_idx]
        
        return waypoints

    def euclidean_distance(self, goal_x, goal_y):
        return sqrt((goal_x - self.pose.x)**2 + (goal_y - self.pose.y)**2)
    
    def linear_velocity(self, goal_x, goal_y, constant=1.5):  # Increased constant
        dist = self.euclidean_distance(goal_x, goal_y)
        return min(constant * dist, self.max_linear_velocity)

    def steering_angle(self, goal_x, goal_y):
        return atan2(goal_y - self.pose.y, goal_x - self.pose.x)
    
    def angular_velocity(self, goal_x, goal_y, constant=6.0):  # Increased constant
        target_angle = self.steering_angle(goal_x, goal_y)
        angle_diff = target_angle - self.pose.theta
        while angle_diff > pi:
            angle_diff -= 2*pi
        while angle_diff < -pi:
            angle_diff += 2*pi
        return np.clip(constant * angle_diff, -self.max_angular_velocity, self.max_angular_velocity)
    
    def orient_to_goal(self, goal_x, goal_y):
        """Rotate turtle in place to face the goal."""
        velocity_msg = Twist()
        target_angle = self.steering_angle(goal_x, goal_y)
        angle_diff = target_angle - self.pose.theta
        
        while angle_diff > pi:
            angle_diff -= 2*pi
        while angle_diff < -pi:
            angle_diff += 2*pi
        
        while abs(angle_diff) >= self.angular_tolerance:
            if rospy.is_shutdown():
                break
            velocity_msg.linear.x = 0
            velocity_msg.angular.z = np.clip(6.0 * angle_diff,  # Increased constant
                                             -self.max_angular_velocity,
                                              self.max_angular_velocity)
            self.velocity_publisher.publish(velocity_msg)
            
            angle_diff = target_angle - self.pose.theta
            while angle_diff > pi:
                angle_diff -= 2*pi
            while angle_diff < -pi:
                angle_diff += 2*pi
            
            self.rate.sleep()

    def move_to_point(self, goal_x, goal_y):
        """Move turtle to a specific (x, y)."""
        velocity_msg = Twist()
        # First, orient
        self.orient_to_goal(goal_x, goal_y)
        
        while self.euclidean_distance(goal_x, goal_y) >= self.distance_tolerance:
            if rospy.is_shutdown():
                break
            velocity_msg.linear.x = self.linear_velocity(goal_x, goal_y)
            velocity_msg.angular.z = self.angular_velocity(goal_x, goal_y)
            self.velocity_publisher.publish(velocity_msg)
            self.rate.sleep()
        
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)
        rospy.loginfo(f"Reached ({goal_x:.2f}, {goal_y:.2f})")


    def draw_m_shape(self):
        """Traverse the approximate contour in order."""
        if not self.waypoints:
            rospy.logerr("No waypoints to draw.")
            return
        
        first_point = self.waypoints[0]
        rospy.loginfo(f"Teleporting to starting point: ({first_point[0]:.2f}, {first_point[1]:.2f})")
        
        self.set_pen(0, 0, 0, 0, 1) 
        rospy.sleep(0.5) 
        
        self.teleport(first_point[0], first_point[1], 0)
        rospy.sleep(0.5)  
        
        self.set_pen(255, 255, 255, 3, 0)  
        rospy.sleep(0.5)  
        
        rospy.loginfo("Starting to draw the 'M' outline...")
        for i, (wx, wy) in enumerate(self.waypoints[1:], 1):  
            rospy.loginfo(f"Waypoint {i+1}/{len(self.waypoints)}: ({wx:.2f}, {wy:.2f})")
            self.move_to_point(wx, wy)
        
        rospy.loginfo(f"Returning to start: ({first_point[0]:.2f}, {first_point[1]:.2f})")
        self.move_to_point(first_point[0], first_point[1])
        
        rospy.loginfo("Done drawing the 'M' shape.")

def main():
    try:
        image_path = "/home/ameyaranade/Desktop/M.png"
        controller = TurtleController(image_path)
        rospy.sleep(0.5)
        controller.draw_m_shape()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

if __name__ == '__main__':
    main()