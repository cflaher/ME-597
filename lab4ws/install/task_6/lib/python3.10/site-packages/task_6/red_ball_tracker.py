import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class RedBallTracker(Node):
    def __init__(self):
        super().__init__('red_ball_tracker')

        # Initialize variables
        self.bbox_msg = None
        self.scan_msg = None
        self.current_distance = float('inf')  # Default to a large value

        # Subscribe to video data
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Subscribe to laser scan data
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Publisher for bounding box information
        self.bbox_publisher = self.create_publisher(
            BoundingBox2D,
            '/bbox',
            10)
        
        # Subscribe to bounding box data
        self.bbox_subscriber = self.create_subscription(
            BoundingBox2D,
            '/bbox',
            self.bbox_callback,
            10)
        
        # Publisher for robot movement commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # Initialize movement command
        self.cmd_msg = Twist()
        self.get_logger().info('Ball follower node started')
        self.timer_period = 0.1  # seconds

        self.create_timer(self.timer_period, self.timer_callback)

        # Initialize CV bridge
        self.bridge = CvBridge()
        self.get_logger().info('Object detector node started')

    def scan_callback(self, msg):
        # Process laser scan data if needed
        # For example, you can use it to avoid obstacles
        #self.get_logger().info(f'Laser scan data received: {msg.ranges}')

        self.current_distance = msg.ranges[0]
        self.scan_msg = msg
        self.get_logger().info(f"distance: {self.current_distance:.2f}")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process image to detect object
        bbox = self.detect_object(cv_image)
        
        if bbox is not None:
            # Extract bounding box information
            x, y, w, h = bbox
            
            # Calculate centroid
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Print object information
            self.get_logger().info(
                f'Object centroid: ({center_x}, {center_y}), width: {w}, height: {h}')
            
            # Create and publish BoundingBox2D message
            bbox_msg = BoundingBox2D()
            bbox_msg.center = Pose2D()
            bbox_msg.center.position.x = float(center_x)
            bbox_msg.center.position.y = float(center_y)
            bbox_msg.center.theta = 0.0  # Orientation (not used for rectangle)
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)
            self.get_logger().info(f'bbox dims: {bbox_msg.size_x}, {bbox_msg.size_y}')
            
            self.bbox_publisher.publish(bbox_msg)
            
            # Draw bounding box on image
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.putText(cv_image, f'({center_x}, {center_y})', 
                       (center_x + 10, center_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Display the edited video feed
        cv2.imshow('Object Detection', cv_image)
        cv2.waitKey(1)

    def bbox_callback(self, msg):
        # Update the bounding box message
        self.bbox_msg = msg

    def detect_object(self, image):
        """
        Detect an object in the image using color-based segmentation.
        Returns bounding box as (x, y, width, height) or None if no object detected.
        """
        # Convert to HSV color space for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for a common color (adjust these values based on your object's color)
        
        '''
        # This example uses red color range
        lower_bound = np.array([0, 120, 70])
        upper_bound = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Red color wraps around in HSV, so we need another range
        lower_bound = np.array([170, 120, 70])
        upper_bound = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_bound, upper_bound)
        '''

        # First range (lower red hues)
        lower_bound1 = np.array([0, 200, 150])     # High saturation and value for brightness
        upper_bound1 = np.array([3, 255, 255])     # Very narrow hue range
        mask1 = cv2.inRange(hsv, lower_bound1, upper_bound1)
        
        # Second range (higher red hues) - red wraps around in HSV
        lower_bound2 = np.array([177, 200, 150])   # High saturation and value for brightness
        upper_bound2 = np.array([180, 255, 255])   # Very narrow hue range
        mask2 = cv2.inRange(hsv, lower_bound2, upper_bound2)
        
        # Combine masks
        mask = mask1 + mask2
        
        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest contour, assuming it's our object of interest
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(largest_contour)
                return (x, y, w, h)
        
        return None
    
    def ball_follower(self, scan_msg, bbox_msg):
        cmd_msg = Twist()

        self.get_logger().info(f"Ball follower running")

        # PID controller parameters  
        # Distance
        kp_dist = 0.1
        ki_dist = 0.0
        kd_dist = 0.0

        # Angle
        kp_heading = 0.0002
        ki_heading = 0.0
        kd_heading = 0.0

        dist_derivative_error = 0.0
        dist_integral_error = 0.0

        heading_derivative_error = 0.0
        heading_integral_error = 0.0

        prev_dist_error = 0.0
        prev_heading_error = 0.0

        if bbox_msg is not None:
            # Extract bounding box information
            center_x = bbox_msg.center.position.x
            center_y = bbox_msg.center.position.y
            w, h = self.bbox_msg.size_x, self.bbox_msg.size_y
            self.get_logger().info(f'bbox dims: ({w}, {h})')
            self.get_logger().info(f'Following object at ({center_x}, {center_y})')

            # Distance control
            dist_error =  (300.0/w) - 1.0
            dist_integral_error = max(min(dist_integral_error + dist_error * self.timer_period, 1), -1)
            dist_derivative_error = (dist_error - prev_dist_error) / self.timer_period
            self.get_logger().info(f'dist error: {dist_error}')
            
            # Heading control
            #heading_error = math.atan2(500.0 - center_x, dist_error)
            heading_error = 500.0 - center_x
            heading_integral_error = max(min(heading_integral_error + heading_error * self.timer_period, 1), -1)
            heading_derivative_error = (heading_error - prev_heading_error) / self.timer_period
            self.get_logger().info(f'heading error: {heading_error}')

            if heading_error < 50.0 and heading_error > -50.0:
                speed = (kp_dist * dist_error +
                        ki_dist * dist_integral_error + 
                        kd_dist * dist_derivative_error)
                heading = 0.0
            else:
                # PID calculations
                speed = 0.0
                heading = (kp_heading * heading_error + 
                        ki_heading * heading_integral_error + 
                        kd_heading * heading_derivative_error)
        else:
            # Stop the robot if no object is detected
            speed = 0.0
            heading = 0.0

            self.get_logger().info('No object detected, stopping robot')

        # # Update previous errors
        # prev_dist_error = dist_error
        # prev_heading_error = heading_error
        
        cmd_msg.linear.x = speed
        cmd_msg.angular.z = heading

        self.get_logger().info(f'Angular velocity: {cmd_msg.angular.z}, Linear velocity: {cmd_msg.linear.x}')

        
        '''

        # Implement ball following logic
        if bbox_msg is not None:
            center_x = bbox_msg.center.position.x
            center_y = bbox_msg.center.position.y
            w, h = self.bbox_msg.size_x, self.bbox_msg.size_y
            self.get_logger().info(f'bbox dims: ({w}, {h})')
            
            # Simple control logic to move towards the object
            if center_x < 300:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.1
            elif center_x > 700:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = -0.1
            else:
                if w < 100.0 and h < 100.0:
                    cmd_msg.linear.x = 0.1
                    cmd_msg.angular.z = 0.0
                else:
                    cmd_msg.linear.x = 0.0
                    cmd_msg.angular.z = 0.0
        

            self.get_logger().info(f'Following object at ({center_x}, {center_y})')
            self.get_logger().info(f'Angular velocity: {cmd_msg.angular.z}, Linear velocity: {cmd_msg.linear.x}')
        else:
            # Stop the robot if no object is detected
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

            self.get_logger().info('No object detected, stopping robot')
        
        '''

        return cmd_msg
    
    def timer_callback(self):
        

        cmd_msg = self.ball_follower(self.scan_msg, self.bbox_msg)
        # Publish the movement command
        self.cmd_publisher.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    red_ball_tracker = RedBallTracker()
    
    try:
        rclpy.spin(red_ball_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cv2.destroyAllWindows()
        red_ball_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
