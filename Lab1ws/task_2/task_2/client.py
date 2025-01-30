import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState

class JointStateClient(Node):
    def __init__(self):
        super().__init__('joint_state_client')
        self.cli = self.create_client(JointState, 'joint_service')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to be available...')

    def send_request(self, x, y, z):
        request = JointState.Request()
        request.x = x
        request.y = y
        request.z = z
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = JointStateClient()
    x, y, z = 1.0, -2.0, 1.5  # Example values
    response = node.send_request(x, y, z)
    node.get_logger().info(f'Sent: x={x}, y={y}, z={z} | Received: Valid={response.valid}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
