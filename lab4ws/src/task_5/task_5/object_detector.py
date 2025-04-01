import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        # Subscribe to video data
        self.subscription = self.create_subscription(
            Image,
            '/video_data',
            self.image_callback,
            10)
        
        # Publisher for bounding box information
        self.bbox_publisher = self.create_publisher(
            BoundingBox2D,
            '/bbox',
            10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        self.get_logger().info('Object detector node started')

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
            bbox_msg.center.x = float(center_x)
            bbox_msg.center.y = float(center_y)
            bbox_msg.center.theta = 0.0  # Orientation (not used for rectangle)
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)
            
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

    def detect_object(self, image):
        """
        Detect an object in the image using color-based segmentation.
        Returns bounding box as (x, y, width, height) or None if no object detected.
        """
        # Convert to HSV color space for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for a common color (adjust these values based on your object's color)
        # This example uses red color range
        lower_bound = np.array([0, 120, 70])
        upper_bound = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Red color wraps around in HSV, so we need another range
        lower_bound = np.array([170, 120, 70])
        upper_bound = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_bound, upper_bound)
        
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

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    
    try:
        rclpy.spin(object_detector)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cv2.destroyAllWindows()
        object_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()