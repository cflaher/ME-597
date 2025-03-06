#!/usr/bin/env python3

import sys
import os
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Navigation(Node):
    """! Navigation node class.
    This class should serve as a template to implement the path planning and
    path follower components to move the turtlebot from position A to B.
    """

    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        super().__init__(node_name)
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.start_time = 0.0

        # Subscribers
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time',10) #DO NOT MODIFY

        # Node rate
        self.rate = self.create_rate(10)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        self.get_logger().info(
            'goal_pose: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        self.ttbot_pose = data.pose
        self.get_logger().info(
            'ttbot_pose: {:.4f}, {:.4f}'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))

    def a_star_path_planner(self, start_pose, end_pose):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        path = Path()
        self.get_logger().info(
            'A* planner.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))
        self.start_time = self.get_clock().now().nanoseconds*1e-9 #Do not edit this line (required for autograder)
        # TODO: IMPLEMENTATION OF THE A* ALGORITHM
        path.poses.append(start_pose)
        path.poses.append(end_pose)
        # Do not edit below (required for autograder)
        self.astarTime = Float32()
        self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9-self.start_time)
        self.calc_time_pub.publish(self.astarTime)
        
        return path

    def get_path_idx(self, path, vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position in the path pointing to the next goal pose to follow.
        """
        idx = 0
        # TODO: IMPLEMENT A MECHANISM TO DECIDE WHICH POINT IN THE PATH TO FOLLOW idx <= len(path)
        for idx in range(len(path.poses)):
            if path.poses[idx+2] == vehicle_pose: #arbitrary index + 2
                break
        return idx

    def path_follower(self, vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        speed = 0.0
        heading = 0.0
        
        # TODO: IMPLEMENT PATH FOLLOWER


        class PidController(Node):

            def __init__(self):
                super().__init__('pid_speed_controller')

                # pid params
                target_pose = self.get_path_idx(path, vehicle_pose) #TODO: DEFINE TARGET DISTANCE
                self.kp = 0.5
                self.ki = 0.0001
                self.kd = 0.15

                target_heading =

                #anti-windup limit
                self.integral_limit = 0.1

                # subscriber
                self.subscription = self.create_subscription(
                    LaserScan,
                    '/scan',
                    self.lidar_callback,
                    10)
                self.subscription  # prevent unused variable warning

                #publisher
                self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
                self.timer_period = 0.1  # seconds
                self.timer = self.create_timer(self.timer_period, self.control)

                self.start_time = self.get_clock().now().nanoseconds * 1e-9

                #self.get_logger().info("PID Controller started")
                

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
                self.integral_error = max(min(self.integral_error, self.integral_limit), -self.integral_limit) # anti-windup
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

        return speed, heading

    def move_ttbot(self, speed, heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired speed.
        @param  heading   Desired yaw angle.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        # TODO: IMPLEMENT YOUR LOW-LEVEL CONTROLLER
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading

        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        while rclpy.ok():
            # Call the spin_once to handle callbacks
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks without blocking

            # 1. Create the path to follow
            path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            # 2. Loop through the path and move the robot
            idx = self.get_path_idx(path, self.ttbot_pose)
            current_goal = path.poses[idx]
            speed, heading = self.path_follower(self.ttbot_pose, current_goal)
            self.move_ttbot(speed, heading)

            self.rate.sleep()
            # Sleep for the rate to control loop timing


def main(args=None):
    rclpy.init(args=args)
    nav = Navigation(node_name='Navigation')

    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()