import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32
from task_3_interfaces.sensor_msgs import LaserScan
from geometry_msgs import Twist

class PID_controller(Node):

    def __init__(self):
        super().__init__('PID_controller')

        # global params
        self.target_dist = 0.35
        self.kp = 0.9
        self.ki = 0.9
        self.kd = 0.9

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg):
        self.current_distance = msg.ranges[0]

    def control(self, cmd)
        cmd = Twist()
        cmd.linear.x = self.output
        self.vel_publisher.publish(cmd)


        dist = LaserScan()
        time = LaserScan()
        
        dist.ranges[0] = Float32()
        time.time_increment[] = Float32()
        
        self.error = 0
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0

        self.error = self.target_dist - pos
        self.integral_error += self.error * time.time_increment
        self.derivative_error = (self.error - self.error_last) / time.time_increment
        self.error_last = self.error
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        return self.output

        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    def __init__(self):
        super().__init__('minimal_publisher')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        dist.data = 'Hello World: %d' % self.i
        self.publisher_.publish(linear.x)
        self.get_logger().info('Publishing: "%s"' % Twist.msg)
        self.i += 1

    def send_vel(self, cmd)
        cmd = Twist()
        cmd.linear.x = self.output
        self.vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()