import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState

class JointService(Node):
    def __init__(self):
        super().__init__('joint_service')
        self.srv = self.create_service(JointState, 'joint_service', self.service_callback)

    def service_callback(self, request, response):
        response.valid = (request.x + request.y + request.z) >= 0
        self.get_logger().info(f'Received: x={request.x}, y={request.y}, z={request.z} -> valid: {response.valid}')
        return response

def main():
    rclpy.init()
    node = JointService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
