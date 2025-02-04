import sys
from task_2_interfaces.srv import JointState
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(JointState, 'joint_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = JointState.Request()

    def send_request(self, x, y, z):
        self.req.x = float(x)
        self.req.y = float(y)
        self.req.z = float(z)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 4:
        print("Usage: script_name x y z")
        return

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(
        float(sys.argv[1]), 
        float(sys.argv[2]), 
        float(sys.argv[3])
    )
    
    minimal_client.get_logger().info(
        'Result of joint_service: for x=%f, y=%f, z=%f is valid=%s' %
        (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), str(response.valid))
    )
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()