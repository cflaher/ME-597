#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState

class JointStateServer(Node):
    def __init__(self):
        super().__init__('service')
        self.srv = self.create_service(
            JointState, 
            'joint_service', 
            self.handle_joint_state
        )
        self.get_logger().info('Joint State Server is ready.')

    def handle_joint_state(self, request, response):
        # Calculate sum of joint values
        total = request.x + request.y + request.z
        
        # Set response based on sum
        response.valid = total >= 0
        
        # Log the request and response
        self.get_logger().info(
            f'Request: x={request.x:.2f}, y={request.y:.2f}, z={request.z:.2f}, ' 
            f'Sum={total:.2f}, Response={response.valid}'
        )
        return response

def main():
    rclpy.init()
    server = JointStateServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()