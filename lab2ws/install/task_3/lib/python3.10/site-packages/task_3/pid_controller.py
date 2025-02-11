import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PidController(Node):

    def __init__(self):
        super().__init__('pid_controller')

        # pid params
        self.target_dist = 0.35
        self.kp = 0.9
        self.ki = 0.00001
        self.kd = 0.1

        # subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

        #publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.control)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("PID Controller started")
        

        # pid variables
        self.current_distance = 0
        self.error = 0
        self.integral_error = 0
        self.prev_error = 0
        self.derivative_error = 0
        self.output = 0

    def lidar_callback(self, msg):
        self.current_distance = msg.ranges[0]

        self.get_logger().info(f"distance: {self.current_distance:.2f}")

    def control(self):
        # calculate errors
        self.error = -self.target_dist + self.current_distance
        self.integral_error += self.error * self.timer_period
        self.derivative_error = (self.error - self.prev_error) / self.timer_period
        self.prev_error = self.error
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        
        current_time = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        self.get_logger().info(f"Current ROS time: {current_time:.2f} seconds\n")

        cmd = Twist()
        if self.output > 0.15:
            self.output = 0.15
        elif self.output < -0.15:
            self.output = -0.15
        
        cmd.linear.x = self.output
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)

        self.prev_error = self.error
    
def control_loop(self):
        self.control()

def main(args=None):
    rclpy.init(args=args)

    pid_controller = PidController()

    rclpy.spin(pid_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
