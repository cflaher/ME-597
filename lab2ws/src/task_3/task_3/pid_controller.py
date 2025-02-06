import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32
from task_3_interfaces.sensor_msgs import LaserScan
from geometry_msgs import Twist

class pid_controller(Node):

    def __init__(self):
        super().__init__('pid_controller')

        # global params
        self.target_dist = 0.35
        self.kp = 0.9
        self.ki = 0.1
        self.kd = 0.1

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.current_distance = 0

    def listener_callback(self, msg):
        self.current_distance = msg.ranges[0]

    def control(self, cmd):
    
        self.error = 0
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0

        self.error = self.target_dist - self.current_distance
        self.integral_error += self.error * self.timer_period
        self.derivative_error = (self.error - self.error_last) / self.timer_period
        self.prev_error = self.error
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        
        cmd = Twist()
        if self.output > 0.15:
            self.output -= .1
        elif self.output < -0.15:
            self.output += .01
        
        cmd.linear.x = self.output
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)

        self.prev_error = self.error
    
def main(args=None):
    rclpy.init(args=args)

    pid_controller = pid_controller()

    rclpy.spin(pid_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    