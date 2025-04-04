import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/video_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.show_video = False  # Custom flag to enable or disable video display

        # Get path to the video file
        package_share_dir = get_package_share_directory('task_5')
        root_dir = os.path.abspath(os.path.join(package_share_dir,'..','..','..','..'))
        video_path = os.path.join(root_dir, 'src', 'task_5', 'resource', 'lab3_video.avi')
        self.get_logger().info(f'Loading video from: {video_path}')

        # Open the video file
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video file: {video_path}')
            rclpy.shutdown()

        # Initialize CV bridge for converting between OpenCV and ROS images
        self.bridge = CvBridge()
        self.get_logger().info('CV Bridge initialized successfully')
        self.get_logger().info('Video publisher started')

    def timer_callback(self):
        # Read a frame from the video
        ret, frame = self.cap.read()
        
        # If we reached the end of the video, loop back to beginning
        if not ret:
            self.get_logger().info('End of video reached, restarting from beginning')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Could not read from video after reset')
                return
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Convert the OpenCV image to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # Publish the image
        self.publisher_.publish(ros_image)

        if self.show_video:                         # Custom flag to enable or disable video display below
            cv2.imshow('Video Window', frame)        # Display frame image in 'Window name'
            if cv2.waitKey(1) & 0xFF == ord('q'):   # Refresh window and check for keypress of `q` 
                self.show_video = True              # Set custom flag to False to disable video display 
                cv2.destroyAllWindows()             # Close all cv2.imshow windows

def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Release the video capture
        image_publisher.cap.release()

        image_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()