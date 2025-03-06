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


# jupyter notebook imports
from PIL import Image, ImageOps 

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm


import yaml
import pandas as pd

from copy import copy, deepcopy
import time

class Queue():
    def __init__(self, init_queue = []):
        self.queue = copy(init_queue)
        self.start = 0
        self.end = len(self.queue)-1
    
    def __len__(self):
        numel = len(self.queue)
        return numel
    
    def __repr__(self):
        q = self.queue
        tmpstr = ""
        for i in range(len(self.queue)):
            flag = False
            if(i == self.start):
                tmpstr += "<"
                flag = True
            if(i == self.end):
                tmpstr += ">"
                flag = True
            
            if(flag):
                tmpstr += '| ' + str(q[i]) + '|\n'
            else:
                tmpstr += ' | ' + str(q[i]) + '|\n'
            
        return tmpstr
    
    def __call__(self):
        return self.queue
    
    def initialize_queue(self,init_queue = []):
        self.queue = copy(init_queue)
    
    def sort(self,key=str.lower):
        self.queue = sorted(self.queue,key=key)
        
    def push(self,data):
        self.queue.append(data)
        self.end += 1
    
    def pop(self):
        p = self.queue.pop(self.start)
        self.end = len(self.queue)-1
        return p
    
class Node():
    def __init__(self,name):
        self.name = name
        self.children = []
        self.weight = []
        
    def __repr__(self):
        return self.name
        
    def add_children(self,node,w=None):
        if w == None:
            w = [1]*len(node)
        self.children.extend(node)
        self.weight.extend(w)
    
class Tree():
    def __init__(self,name):
        self.name = name
        self.root = 0
        self.end = 0
        self.g = {}
        self.g_visual = Graph('G')
    
    def __call__(self):
        for name,node in self.g.items():
            if(self.root == name):
                self.g_visual.node(name,name,color='red')
            elif(self.end == name):
                self.g_visual.node(name,name,color='blue')
            else:
                self.g_visual.node(name,name)
            for i in range(len(node.children)):
                c = node.children[i]
                w = node.weight[i]
                #print('%s -> %s'%(name,c.name))
                if w == 0:
                    self.g_visual.edge(name,c.name)
                else:
                    self.g_visual.edge(name,c.name,label=str(w))
        return self.g_visual
    
    def add_node(self, node, start = False, end = False):
        self.g[node.name] = node
        if(start):
            self.root = node.name
        elif(end):
            self.end = node.name
            
    def set_as_root(self,node):
        # These are exclusive conditions
        self.root = True
        self.end = False
    
    def set_as_end(self,node):
        # These are exclusive conditions
        self.root = False
        self.end = True

class MapProcessor():
    def __init__(self,name):
        self.map = Map(name)
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        self.map_graph = Tree(name)
    
    def __modify_map_pixel(self,map_array,i,j,value,absolute):
        if( (i >= 0) and 
            (i < map_array.shape[0]) and 
            (j >= 0) and
            (j < map_array.shape[1]) ):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value 
    
    def __inflate_obstacle(self,kernel,map_array,i,j,absolute):
        dx = int(kernel.shape[0]//2)
        dy = int(kernel.shape[1]//2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array,i,j,kernel[0][0],absolute)
        else:
            for k in range(i-dx,i+dx):
                for l in range(j-dy,j+dy):
                    self.__modify_map_pixel(map_array,k,l,kernel[k-i+dx][l-j+dy],absolute)
        
    def inflate_map(self,kernel,absolute=True):
        # Perform an operation like dilation, such that the small wall found during the mapping process
        # are increased in size, thus forcing a safer path.
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.map.image_array[i][j] == 0:
                    self.__inflate_obstacle(kernel,self.inf_map_img_array,i,j,absolute)
        r = np.max(self.inf_map_img_array)-np.min(self.inf_map_img_array)
        if r == 0:
            r = 1
        self.inf_map_img_array = (self.inf_map_img_array - np.min(self.inf_map_img_array))/r
                
    def get_graph_from_map(self):
        # Create the nodes that will be part of the graph, considering only valid nodes or the free space
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d'%(i,j))
                    self.map_graph.add_node(node)
        # Connect the nodes through edges
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:                    
                    if (i > 0):
                        if self.inf_map_img_array[i-1][j] == 0:
                            # add an edge up
                            child_up = self.map_graph.g['%d,%d'%(i-1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up],[1])
                    if (i < (self.map.image_array.shape[0] - 1)):
                        if self.inf_map_img_array[i+1][j] == 0:
                            # add an edge down
                            child_dw = self.map_graph.g['%d,%d'%(i+1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw],[1])
                    if (j > 0):
                        if self.inf_map_img_array[i][j-1] == 0:
                            # add an edge to the left
                            child_lf = self.map_graph.g['%d,%d'%(i,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_lf],[1])
                    if (j < (self.map.image_array.shape[1] - 1)):
                        if self.inf_map_img_array[i][j+1] == 0:
                            # add an edge to the right
                            child_rg = self.map_graph.g['%d,%d'%(i,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_rg],[1])
                    if ((i > 0) and (j > 0)):
                        if self.inf_map_img_array[i-1][j-1] == 0:
                            # add an edge up-left 
                            child_up_lf = self.map_graph.g['%d,%d'%(i-1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_lf],[np.sqrt(2)])
                    if ((i > 0) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i-1][j+1] == 0:
                            # add an edge up-right
                            child_up_rg = self.map_graph.g['%d,%d'%(i-1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_rg],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j > 0)):
                        if self.inf_map_img_array[i+1][j-1] == 0:
                            # add an edge down-left 
                            child_dw_lf = self.map_graph.g['%d,%d'%(i+1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_lf],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i+1][j+1] == 0:
                            # add an edge down-right
                            child_dw_rg = self.map_graph.g['%d,%d'%(i+1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_rg],[np.sqrt(2)])                    
        
    def gaussian_kernel(self, size, sigma=1):
        size = int(size) // 2
        x, y = np.mgrid[-size:size+1, -size:size+1]
        normal = 1 / (2.0 * np.pi * sigma**2)
        g =  np.exp(-((x**2 + y**2) / (2.0*sigma**2))) * normal
        r = np.max(g)-np.min(g)
        sm = (g - np.min(g))*1/r
        return sm
    
    def rect_kernel(self, size, value):
        m = np.ones(shape=(size,size))
        return m
    
    def draw_path(self,path):
        path_tuple_list = []
        path_array = copy(self.inf_map_img_array)
        for idx in path:
            tup = tuple(map(int, idx.split(',')))
            path_tuple_list.append(tup)
            path_array[tup] = 0.5
        return path_array

class Navigation(Node):
    """! Navigation node class.
    This class should serve as a template to implement the path planning and
    path follower components to move the turtlebot from position A to B.
    """

    def __init__(self, node_name='auto_navigator'):
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
        mp = MapProcessor('map')
        path = Path()
        self.get_logger().info(
            'A* planner.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))
        self.start_time = self.get_clock().now().nanoseconds*1e-9 #Do not edit this line (required for autograder)
        # TODO: IMPLEMENTATION OF THE A* ALGORITHM
        path.poses.append(start_pose)
        path.poses.append(end_pose)

        import numpy as np
        import queue as Queue

        class AStar():
            def __init__(self, in_tree):
                self.in_tree = in_tree
                self.q = Queue.Queue()  # FIFO queue
                self.dist = {name: np.Inf for name, node in in_tree.g.items()}  # g-score
                self.h = {name: 0 for name, node in in_tree.g.items()}  # Heuristic (h-score)
                self.via = {name: None for name, node in in_tree.g.items()}  # Path tracking
                
                # Compute heuristic values (Euclidean distance)
                for name, node in in_tree.g.items():
                    start = tuple(map(int, name.split(',')))
                    end = tuple(map(int, self.in_tree.end.split(',')))
                    self.h[name] = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)

                

            def __get_f_score(self, node):
                """Calculate f = g + h for a node."""
                idx = node.name
                return self.dist[idx] + self.h[idx]

            def solve(self, sn, en):
                """A* pathfinding from start node `sn` to end node `en`."""
                self.dist[sn.name] = 0  # Set start node distance to 0
                self.q.put(sn)  # Enqueue start node

                while not self.q.empty():
                    # Sort queue based on f-score and get the best node
                    nodes = list(self.q.queue)
                    nodes.sort(key=self.__get_f_score)  # Sort by f-score
                    u = nodes.pop(0)  # Pick the best node
                    print(u.name,self.q.queue)
                    self.q.queue.clear()
                    for n in nodes:
                        self.q.put(n)  # Re-add remaining nodes

                    # Stop if we reach the goal
                    if u.name == en.name:
                        break

                    # Explore neighbors
                    for i in range(len(u.children)):
                        c = u.children[i]  # Child node
                        w = u.weight[i]  # Edge weight
                        new_dist = self.dist[u.name] + w  # Compute g-score

                        # If a shorter path is found, update
                        if new_dist < self.dist[c.name]:
                            self.dist[c.name] = new_dist
                            self.via[c.name] = u.name  # Track path
                            self.q.put(c)  # Add neighbor to queue

            def reconstruct_path(self, sn, en):
                """Reconstructs the shortest path from `sn` to `en`."""
                path = []
                u = en.name
                dist = self.dist[u]

                # Backtrack from goal to start
                while u is not None:
                    path.append(u)
                    u = self.via[u]
                
                path.reverse()  # Correct order from start → end
                return path, dist

        mp.map_graph.root = "100,100"
        mp.map_graph.end = "75,120"

        as_maze = AStar(mp.map_graph)

        start = time.time()
        as_maze.solve(mp.map_graph.g[mp.map_graph.root],mp.map_graph.g[mp.map_graph.end])
        end = time.time()
        print('Elapsed Time: %.3f'%(end - start))

        path, dist = as_maze.reconstruct_path(mp.map_graph.g[mp.map_graph.root],mp.map_graph.g[mp.map_graph.end])

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
            if path.poses[idx] == vehicle_pose: #arbitrary distance 5

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


        class DistancePidController(Node):

            def __init__(self):
                super().__init__('pid_speed_controller')

                # pid params
                self.kp = 0.5
                self.ki = 0.0001
                self.kd = 0.15

                self.path = self.a_star_path_planner(vehicle_pose, current_goal_pose)
                self.target_heading = self.get_path_idx(self.path, vehicle_pose) #TODO: DEFINE TARGET HEADING
                self.target_pose = self.get_path_idx(self.path, vehicle_pose) #TODO: DEFINE TARGET DISTANCE
        
                #anti-windup limit
                self.integral_limit = 0.1

                # pid variables
                self.current_distance = 0
                self.error = 0
                self.integral_error = 0
                self.prev_error = 0
                self.derivative_error = 0
                self.output = 0
        
            def control(self):
                # calculate errors
                self.error = np.sqrt((self.goal_pose.pose.position.x - self.ttbot_pose.pose.position.x)**2 + (self.goal_pose.pose.position.y - self.ttbot_pose.pose.position.y)**2)
                self.integral_error += self.error * self.timer_period
                self.integral_error = max(min(self.integral_error, self.integral_limit), -self.integral_limit) # anti-windup
                self.derivative_error = (self.error - self.prev_error) / self.timer_period
                self.prev_error = self.error
                self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
               
                speed = self.output
                self.prev_error = self.error
            
            def control_loop(self):
                    self.control()
        
        class HeadingPidController(Node):

            def __init__(self):
                super().__init__('pid_speed_controller')

                # pid params
                self.kp = 0.5
                self.ki = 0.0001
                self.kd = 0.15

                self.path = self.a_star_path_planner(vehicle_pose, current_goal_pose)
                self.target_heading = self.get_path_idx(self.path, vehicle_pose) #TODO: DEFINE TARGET HEADING
                self.target_pose = self.get_path_idx(self.path, vehicle_pose) #TODO: DEFINE TARGET DISTANCE
                
                #anti-windup limit
                self.integral_limit = 0.1
                
                # pid variables
                self.current_distance = 0
                self.error = 0
                self.integral_error = 0
                self.prev_error = 0
                self.derivative_error = 0
                self.output = 0

            def control(self):
                # calculate errors
                self.error = -self.goal_pose.pose.orientation.w + self.ttbot_pose.pose.orientation.w

                self.integral_error += self.error * self.timer_period
                self.integral_error = max(min(self.integral_error, self.integral_limit), -self.integral_limit) # anti-windup
                self.derivative_error = (self.error - self.prev_error) / self.timer_period
                self.prev_error = self.error
                self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
                
                # current_time = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
                # self.get_logger().info(f"Current ROS time: {current_time:.2f} seconds\n")

                cmd = Twist()
                if self.output > 0.15:
                    self.output = 0.15
                elif self.output < -0.15:
                    self.output = -0.15
                
                heading = self.output
                self.prev_error = self.error
            
            def control_loop(self):
                    self.control()

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