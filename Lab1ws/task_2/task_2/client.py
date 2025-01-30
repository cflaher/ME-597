import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState

class JointStateClient(Node):
    def __init__(self):
        super().__init__('client')
        self.client = self.create_client(JointState, 'joint_service')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, x, y, z):
        request = JointState.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = JointStateClient()
    
    test_values = [
        (1.0, 2.0, 3.0),    
        (-1.0, -2.0, -3.0), 
        (1.0, -1.0, 0.0),   
    ]
    
    try:
        for x, y, z in test_values:
            future = client.send_request(x, y, z)
            rclpy.spin_until_future_complete(client, future)
            
            if future.done():
                try:
                    response = future.result()
                    client.get_logger().info(
                        f'Test case (x={x}, y={y}, z={z}): '
                        f'Response={response.valid}'
                    )
                except Exception as e:
                    client.get_logger().error(f'Service call failed: {str(e)}')
            else:
                client.get_logger().error('Service call timed out')
                
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
