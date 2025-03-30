#!/usr/bin/env python3

import sys
import os
import numpy as np
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from std_msgs.msg import Float32
from transforms3d.euler import quat2euler
import time
from task_4.utilts.MapProcessor import MapProcessor  
from task_4.utilts.Astar import AStar        


class Navigation(Node):
    """Navigation node for autonomous path planning and path following."""

    def __init__(self, node_name='Navigator'):
        """Class constructor to set up the node.
        @param  node_name   Name of the ROS node.
        """
        super().__init__(node_name)
        self.path = Path()
        self.goal_pose = PoseStamped()       
        self.ttbot_pose = PoseStamped()     
        self.start_time = 0.0     
        self.is_goal_set = False 

        # Subscribers
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time',10) #DO NOT MODIFY
        self.rate = self.create_rate(10)  # 10Hz


        #Map
        map_name = 'sync_classroom_map'
        self.mp = MapProcessor(map_name)
        kr = self.mp.rect_kernel(8, 1)
        self.mp.inflate_map(kr, True)
        self.mp.get_graph_from_map()
        self.get_logger().info('Map:'+map_name + 'loaded')

        

    def world_to_pixel(self, x, y, min_x=-5.4, max_y=4.2, scale_x=13.42,scale_y=13.49):
        # Convert world coordinates to pixel coordinates
        x_origin, y_origin = 72,56  # Must match pixel_to_world
        resolution = 1/0.075
        px = int(round(x_origin + x * resolution))
        py = int(round(y_origin - y * resolution))
        return px, py

    def pixel_to_world(self, px, py, min_x=-5.4, max_y=4.2, scale_x=0.0745,scale_y=0.0741):
        # Convert pixel coordinates to world coordinates
        x_origin, y_origin = 72,56  # Must match world_to_pixel
        resolution = 1/0.075
        world_x = (px - x_origin) / resolution
        world_y = (y_origin - py) / resolution
        return world_x, world_y

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        self.get_logger().info('Received goal: (%.2f, %.2f)' % 
                                 (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))
        self.is_goal_set = True

    def __ttbot_pose_cbk(self, data):
        
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
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            'A* path planner. \n start: {}, \n end: {}'.format(
                (start_pose.pose.position.x, start_pose.pose.position.y),
                (end_pose.pose.position.x, end_pose.pose.position.y)))
        self.start_time = self.get_clock().now().nanoseconds * 1e-9 # Do not modify, Start time for A* computation


        start_pose = self.world_to_pixel(start_pose.pose.position.x, start_pose.pose.position.y)
        end_pose = self.world_to_pixel(end_pose.pose.position.x, end_pose.pose.position.y)
        self.mp.map_graph.root = str(start_pose[1]) + ',' + str(start_pose[0])
        self.mp.map_graph.end = str(end_pose[1]) + ',' + str(end_pose[0])
        self.get_logger().info('start: {}, end: {}'.format(self.mp.map_graph.root, self.mp.map_graph.end))

        as_maze = AStar(self.mp.map_graph)
        as_maze.solve(self.mp.map_graph.g[self.mp.map_graph.root], self.mp.map_graph.g[self.mp.map_graph.end])
        path_as, dist_as = as_maze.reconstruct_path(self.mp.map_graph.g[self.mp.map_graph.root], self.mp.map_graph.g[self.mp.map_graph.end])
        self.get_logger().info('Path: {}'.format(path_as))

        for node in path_as:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            world_x, world_y = self.pixel_to_world(int(node.split(',')[1]), int(node.split(',')[0]))
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            path.poses.append(pose)

        elapsed_time = Float32()
        elapsed_time.data = self.get_clock().now().nanoseconds * 1e-9 - self.start_time # Do not modify, End time for A* computation
        self.calc_time_pub.publish(elapsed_time)
        self.path_pub.publish(path)

        return path

    def get_path_idx(self, path, vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position in the path pointing to the next goal pose to follow.
        """
        idx = 0
        min_dist = float('inf')
        
        current_pose = np.array([vehicle_pose.pose.position.x, vehicle_pose.pose.position.y])
        for i, pose in enumerate(path.poses):
            waypoint = np.array([pose.pose.position.x, pose.pose.position.y])
            dist = np.linalg.norm(waypoint - current_pose)
            if dist < min_dist:
                min_dist = dist
                idx = i
            
        # 2. Look ahead with dynamic threshold
        lookahead_dist = 0.5
        for i in range(idx, len(path.poses)):
            waypoint = np.array([path.poses[i].pose.position.x, path.poses[i].pose.position.y])
            dist = np.linalg.norm(waypoint - current_pose)
            if dist > lookahead_dist:
                idx = i
                break
        return idx

    def path_follower(self, vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  current_pose       PoseStamped object containing the current vehicle pose.
        @param  target_pose   String containing the name of the target node (e.g., "i,j").
        @return speed              Float containing the linear speed of the robot.
        @return heading            Float containing the angular heading of the robot.
        """
        current_x = vehicle_pose.pose.position.x
        current_y = vehicle_pose.pose.position.y

        target_x = current_goal_pose.pose.position.x
        target_y = current_goal_pose.pose.position.y
        dx = target_x - current_x
        dy = target_y - current_y
        dist= math.sqrt(dx * dx + dy * dy)  
        heading = math.atan2(dy, dx)  

        self.get_logger().info(f"distance: {dist:.2f}, heading: {heading:.2f}")


        speed = 0.0
        angular_speed = 0.0
        dist_threshold = 0.2  
        if dist > dist_threshold:
            speed = 0.15  
            current_yaw = self.get_yaw_from_pose(vehicle_pose)  
            angle_diff = heading - current_yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            angular_speed = 0.7 * angle_diff  

        return speed, angular_speed
    
    def get_yaw_from_pose(self, posestamp_curr_angle):
        """! Helper function to extract yaw from a PoseStamped object.
        @param  pose    PoseStamped object containing the pose.
        @return yaw     Float containing the yaw angle in radians.
        """
        quat = [posestamp_curr_angle.pose.orientation.w, posestamp_curr_angle.pose.orientation.x, posestamp_curr_angle.pose.orientation.y, posestamp_curr_angle.pose.orientation.z]
        roll, pitch, yaw = quat2euler(quat, axes='sxyz')
        return yaw

    def move_ttbot(self, speed, angular_speed):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired speed.
        @param  heading   Desired yaw angle.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        path = Path()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)  

            if self.is_goal_set:
                self.is_goal_set = False
                path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)

            if path.poses != []:
                idx = self.get_path_idx(path, self.ttbot_pose)
                current_goal = path.poses[idx]
            
                speed, angular_speed = self.path_follower(self.ttbot_pose, current_goal)
                self.move_ttbot(speed, angular_speed)

            time.sleep(0.1)

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

