import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState

class JointClient(Node):
    def __init__(self):
        super().__init__('joint_client')
        self.client = self.create_client(JointState, 'joint_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.request = JointState.Request()

    def send_request(self, x, y, z):
        self.request.x = x
        self.request.y = y
        self.request.z = z
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = JointClient()
    response = node.send_request(1.0, -2.0, 2.0)
    node.get_logger().info(f'Service response: {response.valid}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
