import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState

class JointStateService(Node):
    def __init__(self):
        super().__init__('joint_state_service')
        self.srv = self.create_service(JointState, 'joint_service', self.process_request)
        self.get_logger().info('JointState service is ready.')

    def process_request(self, request, response):
        total = request.x + request.y + request.z
        response.valid = total >= 0  # True if sum is >= 0, False otherwise
        self.get_logger().info(f'Received: x={request.x}, y={request.y}, z={request.z} | Sum={total} | Valid={response.valid}')
        return response

def main():
    rclpy.init()
    node = JointStateService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
