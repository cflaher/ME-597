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
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# jupyter notebook imports
from PIL import Image, ImageOps 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from graphviz import Graph
import heapq

import yaml
import pandas as pd

from copy import copy, deepcopy
import time
from ament_index_python.packages import get_package_share_directory

class Map():
    def __init__(self, map_name):
        self.map_im, self.map_df, self.limits = self.__open_map(map_name)
        self.image_array = self.__get_obstacle_map(self.map_im, self.map_df)

    def __repr__(self):
        fig, ax = plt.subplots(dpi=150)
        ax.imshow(self.image_array,extent=self.limits, cmap=cm.gray)
        ax.plot()
        return ""

    def __open_map(self, map_name):
        # Open the YAML file which contains the map name and other
        # configuration parameters
        package_name = "task_4"
        package_path = get_package_share_directory(package_name)
        
        # Path to the map YAML file
        map_path = "/home/me597/ME-597/lab3ws"
        map_yaml_path = os.path.join(map_path, map_name + '.yaml')
        
        # Open the YAML file
        with open(map_yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)
            map_df = pd.json_normalize(map_data)
        
        # Get the path to the map image
        # The image path in the YAML might be relative to the YAML file location
        # So we need to construct the absolute path
        map_image_name = map_df.image[0]
        
        # If the image path in YAML is absolute, use it directly
        if os.path.isabs(map_image_name):
            map_image_path = map_image_name
        else:
            # If it's relative, assume it's relative to the YAML file's directory
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_name)
        
        # Check if the file exists
        if not os.path.exists(map_image_path):
            # Try alternative locations
            alt_path = os.path.join(map_path, map_image_name)
            if os.path.exists(alt_path):
                map_image_path = alt_path
        
        # Open the map image
        im = Image.open(map_image_path)
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        
        # Get the limits of the map
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]
        
        return im, map_df, [xmin, xmax, ymin, ymax]

    def __get_obstacle_map(self,map_im, map_df):
        img_array = np.reshape(list(self.map_im.getdata()),(self.map_im.size[1],self.map_im.size[0]))
        up_thresh = self.map_df.occupied_thresh[0]*255
        low_thresh = self.map_df.free_thresh[0]*255

        for j in range(self.map_im.size[0]):
            for i in range(self.map_im.size[1]):
                if img_array[i,j] > up_thresh:
                    img_array[i,j] = 255
                else:
                    img_array[i,j] = 0
        return img_array

class AStarQueue():
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
    
class GraphNode():
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
                    node = GraphNode('%d,%d'%(i,j))
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
    
    # Graph debugging

    def print_graph_nodes(self):
    #     """Print all available nodes in the graph."""
    #     print("Available Graph Nodes:")
    #     for node_name, node in self.map_graph.g.items():
    #         print(f"Node: {node_name}")
    #         print(f"  Children: {[child.name for child in node.children]}")
    #         print(f"  Weights:  {node.weight}")
    #         print("---")
        
         print(f"\nTotal number of nodes: {len(self.map_graph.g)}")    

import numpy as np
import queue as Queue

class AStar():
    def __init__(self, in_tree):
        self.in_tree = in_tree
        self.q = Queue.Queue()  # FIFO queue
        self.dist = {name: np.inf for name, node in in_tree.g.items()}  # g-score
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


class Navigation(Node):
    """! Navigation node class.
    This class should serve as a template to implement the path planning and
    path follower components to move the turtlebot from position A to B.
    """

    def __init__(self, map_processor, node_name='Navigation'):
        """! Class constructor.
        @param  map_processor  An instance of the MapProcessor class with the inflated map.
        @param  node_name      Name of the ROS node.
        @return An instance of the Navigation class.
        """
        super().__init__(node_name)
        
        # Store map processor
        self.map_processor = map_processor
        
        # Get the graph from map processor
        self.graph = map_processor.map_graph
        
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.start_time = 0.0
        self.timer_period = 0.1  # 10Hz operation

        # pid distance variables
        self.current_distance = 0
        self.dist_error = 0
        self.dist_integral_error = 0
        self.prev_dist_error = 0
        self.dist_derivative_error = 0
        self.dist_output = 0

        # pid heading variables
        self.current_heading = 0
        self.heading_error = 0
        self.heading_integral_error = 0
        self.prev_heading_error = 0
        self.heading_derivative_error = 0
        self.heading_output = 0

        # Subscribers
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time',10) #DO NOT MODIFY

        # Node rate
        self.rate = self.create_rate(10)

        # A* path planner variables
        self.q = AStarQueue()  # FIFO queue
        self.dist = {name: np.Inf for name, node in self.graph.g.items()}  # g-score
        self.h = {name: 0 for name, node in self.graph.g.items()}  # Heuristic (h-score)
        self.via = {name: None for name, node in self.graph.g.items()}  # Path tracking
        
        # Map-related constants
        # self.resolution = self.map_processor.map.map_df.resolution[0]
        # self.offset_x = self.map_processor.map.map_df.origin[0][0]
        # self.offset_y = self.map_processor.map.map_df.origin[0][1]

        self.resolution = 15.03759398
        self.offset_y = 4.33
        self.offset_x = -3.81

    def __goal_pose_cbk(self, msg):
        """Callback for receiving goal pose"""
        self.goal_pose = msg
        self.get_logger().info(f'Received goal pose: {msg.pose.position}')

    def __ttbot_pose_cbk(self, msg):
        """Callback for receiving current robot pose"""
        # Convert PoseWithCovarianceStamped to PoseStamped
        self.ttbot_pose = PoseStamped()
        self.ttbot_pose.header = msg.header
        self.ttbot_pose.pose = msg.pose.pose
        self.get_logger().info(f'Received robot pose: {msg.pose.pose.position}')    

    import heapq
    import numpy as np

    def a_star_path_planner(self, start_pose, end_pose):
        """! Optimized A* path planner with improved performance."""
        path = Path()
        path.header.frame_id = "map"
        
        # Logging and timing
        self.get_logger().info(
            'A* planner.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))
        self.start_time = self.get_clock().now().nanoseconds*1e-9
        
        # Coordinate conversion
        def world_to_map_coords(x, y):
            """Convert world coordinates to map coordinates."""
            map_x = int((-y + self.offset_y) * self.resolution)
            map_y = int((x - self.offset_x) * self.resolution)
        
            node_name = f"{map_x},{map_y}"
            
            self.get_logger().info(f"Converting (x,y): ({x},{y}) to map coords: {node_name}")
            self.get_logger().info(f"Is {node_name} in graph: {node_name in self.graph.g}")

            return f"{map_x},{map_y}"
            
        # Convert start and end coordinates
        start_node_name = world_to_map_coords(
            start_pose.pose.position.x,
            start_pose.pose.position.y
        )
        end_node_name = world_to_map_coords(
            end_pose.pose.position.x,
            end_pose.pose.position.y
        )
        
        # Find valid nodes
        def find_valid_node(node_name):
            """Find the nearest valid node efficiently."""
            if node_name in self.graph.g:
                return node_name
            
            # Faster node search with limited radius
            x, y = map(int, node_name.split(','))
            search_radius = 5  # Configurable search radius
            
            for r in range(search_radius):
                for dx in range(-r, r+1):
                    for dy in range(-r, r+1):
                        candidate = f"{x+dx},{y+dy}"
                        if candidate in self.graph.g:
                            return candidate
            
            return None
        
        # Validate and find valid start/end nodes
        start_node_name = find_valid_node(start_node_name)
        end_node_name = find_valid_node(end_node_name)
        
        if not start_node_name or not end_node_name:
            self.get_logger().error("Invalid start or end nodes")
            return path
        
        # Precompute coordinates for heuristic
        start_coords = tuple(map(int, start_node_name.split(',')))
        end_coords = tuple(map(int, end_node_name.split(',')))
        
        # Efficient heuristic function (Manhattan distance)
        def heuristic(node_name):
            """Compute heuristic using Manhattan distance."""
            node_coords = tuple(map(int, node_name.split(',')))
            return abs(node_coords[0] - end_coords[0]) + abs(node_coords[1] - end_coords[1])
        
        # Efficient A* implementation using heapq
        open_set = []
        heapq.heappush(open_set, (0, start_node_name))
        
        # Efficient data structures
        came_from = {}
        g_score = {start_node_name: 0}
        f_score = {start_node_name: heuristic(start_node_name)}
        
        # Limit iterations to prevent infinite loops
        max_iterations = 1000
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # Get node with lowest f_score
            current_f, current_node = heapq.heappop(open_set)
            
            # Goal reached
            if current_node == end_node_name:
                break
            
            # Get current graph node
            current_graph_node = self.graph.g[current_node]
            
            # Explore neighbors
            for i, neighbor in enumerate(current_graph_node.children):
                neighbor_name = neighbor.name
                tentative_g_score = g_score[current_node] + current_graph_node.weight[i]
                
                # Better path found
                if (neighbor_name not in g_score or
                    tentative_g_score < g_score.get(neighbor_name, float('inf'))):
                    
                    # Update path tracking
                    came_from[neighbor_name] = current_node
                    g_score[neighbor_name] = tentative_g_score
                    f_score[neighbor_name] = tentative_g_score + heuristic(neighbor_name)
                    
                    # Add to open set
                    heapq.heappush(open_set, (f_score[neighbor_name], neighbor_name))
        

        def reconstruct_path(came_from, current):
            total_path = [current]
            while current in came_from:
                current = came_from[current]
                total_path.append(current)
            return list(reversed(total_path))
        
        if end_node_name in came_from or end_node_name == start_node_name:
            node_path = reconstruct_path(came_from, end_node_name)
        else:
            self.get_logger().error("No path found")
            return path
        
        # Debug: Path statistics
        self.get_logger().info(f"\nPath Planning Results:")
        self.get_logger().info(f"Total nodes in path: {len(node_path)}")
        self.get_logger().info(f"Path nodes: {node_path[:3]}...{node_path[-3:]}")

        
        # Convert to ROS Path message
        for node_name in node_path:
            x, y = map(int, node_name.split(','))
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = (y / self.resolution) + self.offset_x
            pose.pose.position.y = -(x / self.resolution) + self.offset_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        print(f"Path: {path}")
        
        # Do not edit below (required for autograder)
        self.astarTime = Float32()
        self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9-self.start_time)
        self.calc_time_pub.publish(self.astarTime)
        
        return path

        
    def get_path_idx(self, path, vehicle_pose):
        """Find the next waypoint to follow in the path."""
        if not path.poses:
            return 0
        
        # Track the last known index to prevent backtracking
        if not hasattr(self, 'last_path_idx'):
            self.last_path_idx = 0
        
        # Find the furthest reachable point ahead
        max_look_ahead = 1.0  # meters to look ahead
        target_idx = self.last_path_idx
        
        for i in range(self.last_path_idx, len(path.poses)):
            # Calculate distance to this waypoint
            dx = path.poses[i].pose.position.x - vehicle_pose.pose.position.x
            dy = path.poses[i].pose.position.y - vehicle_pose.pose.position.y
            distance = np.sqrt(dx*dx + dy*dy)
            
            # Break if we exceed look-ahead distance
            if distance > max_look_ahead:
                break
            
            # Update best index
            target_idx = i
            print(f"Closest waypoint: {path.poses[target_idx].pose.position}")
        
        # Update last known index
        self.last_path_idx = target_idx

        return target_idx

    def path_follower(self, vehicle_pose, current_goal_pose):
        """! Path follower with improved stability and debugging.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path.
        @return speed, heading         Tuple containing linear speed and angular heading commands
        """
        # Debugging flag - set to True to print detailed information
        DEBUG = True

        # Prevent PID windup and ensure clean initialization
        if not hasattr(self, 'prev_dist_error'):
            self.prev_dist_error = 0.0
            self.dist_integral_error = 0.0
            
        if not hasattr(self, 'prev_heading_error'):
            self.prev_heading_error = 0.0
            self.heading_integral_error = 0.0

        # More aggressive PID parameters with anti-windup
        kp_dist = 0.5      # Increased proportional gain for distance
        ki_dist = 0.0      # Reduced integral gain
        kd_dist = 0.2      # Moderate derivative gain
        
        kp_heading = 0.1   # Increased proportional gain for heading
        ki_heading = 0.0   # Reduced integral gain
        kd_heading = 0.0   # Moderate derivative gain

        # Calculate distance to goal
        dx = current_goal_pose.pose.position.x - vehicle_pose.pose.position.x
        dy = current_goal_pose.pose.position.y - vehicle_pose.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate target heading (angle to goal in world frame)
        target_heading = math.atan2(dy, dx)

        # Convert quaternion to yaw
        def quaternion_to_yaw(q):
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

        current_yaw = quaternion_to_yaw(vehicle_pose.pose.orientation)
        self.get_logger().info(f"Current Yaw: {math.degrees(current_yaw):.2f}°")

        # Calculate heading error with advanced normalization
        heading_error = target_heading - current_yaw
        self.get_logger().info(f"Heading error: {heading_error}")

        # Distance control
        self.dist_error = distance
        self.dist_integral_error = max(min(self.dist_integral_error + self.dist_error * self.timer_period, 1), -1)
        self.dist_derivative_error = (self.dist_error - self.prev_dist_error) / self.timer_period
        
        # Heading control
        self.heading_error = heading_error
        self.heading_integral_error = max(min(self.heading_integral_error + self.heading_error * self.timer_period, 1), -1)
        self.heading_derivative_error = (self.heading_error - self.prev_heading_error) / self.timer_period

        # PID calculations with additional safety checks
        speed = (kp_dist * self.dist_error + 
                ki_dist * self.dist_integral_error + 
                kd_dist * self.dist_derivative_error)

        heading = (kp_heading * self.heading_error + 
                ki_heading * self.heading_integral_error + 
                kd_heading * self.heading_derivative_error)

        # Aggressive limits to prevent excessive turning
        max_speed = 0.3      # m/s (slightly increased)
        max_heading = 1.0    # rad/s (with more control)
        
        # Add heading priority when far from goal
        if distance > 0.5:
            # Prioritize heading correction
            speed = min(speed, 0.1)  # Slow down while correcting heading
        
        if heading_error > 10:
            speed = 0.0  # Stop if heading error is too large
            
        # Apply limits with additional safety
        #speed = max(min(speed, max_speed), -max_speed)
        #heading = max(min(heading, max_heading), -max_heading)

        # Near goal behavior
        goal_tolerance = 0.15  # 15 cm tolerance
        if distance < goal_tolerance:
            speed = 0.0
            heading = 0.0

        # Debugging output
        if DEBUG:
            print(f"Distance to goal: {distance:.3f} m")
            print(f"Current Yaw: {math.degrees(current_yaw):.2f}°")
            print(f"Target Heading: {math.degrees(target_heading):.2f}°")
            print(f"Heading Error: {math.degrees(heading_error):.2f}°")
            print(f"Computed Speed: {speed:.3f} m/s")
            print(f"Computed Heading: {math.degrees(heading):.3f}°/s")
            print("---")

        # Update previous errors
        self.prev_dist_error = self.dist_error
        self.prev_heading_error = self.heading_error

        return speed, heading

    def move_ttbot(self, speed, heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired speed.
        @param  heading   Desired yaw angle.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        
        cmd_vel.angular.z = heading
        cmd_vel.linear.x = speed
        
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """

        # Initialize variables
        has_goal = False
        path = Path()
        
        while rclpy.ok():
            # Process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check for valid poses
            if not hasattr(self, 'ttbot_pose') or not hasattr(self, 'goal_pose'):
                self.get_logger().warn('Waiting for valid pose information...')
                continue
            
            # Check for non-zero coordinates
            if (self.ttbot_pose.pose.position.x == 0 and 
                self.ttbot_pose.pose.position.y == 0 and 
                self.goal_pose.pose.position.x == 0 and 
                self.goal_pose.pose.position.y == 0):
                self.get_logger().warn('Received zero coordinates. Waiting for valid poses...')
                continue    
            
            # If we receive a new goal, plan a new path
            if not has_goal and self.goal_pose.header.stamp != rclpy.time.Time():
                self.get_logger().info('New goal received, planning path...')
                path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
                self.path_pub.publish(path)
                has_goal = True
            
            # Follow path if we have one
            if has_goal and len(path.poses) > 0:
                idx = self.get_path_idx(path, self.ttbot_pose)
                
                # Check if we reached the final goal
                if idx == len(path.poses) - 1:
                    # Check if we're close enough to the final goal
                    dx = path.poses[-1].pose.position.x - self.ttbot_pose.pose.position.x
                    dy = path.poses[-1].pose.position.y - self.ttbot_pose.pose.position.y
                    if np.sqrt(dx*dx + dy*dy) < 0.1:  # 10cm tolerance
                        self.get_logger().info('Goal reached!')
                        self.move_ttbot(0.0, 0.0)  # Stop the robot
                        has_goal = False
                        continue
                
                # Follow the path
                #current_goal = path.poses[idx]
                current_goal = self.goal_pose
                self.get_logger().info(f"Current goal: {current_goal.pose.position}")
                speed, heading = self.path_follower(self.ttbot_pose, current_goal)
                self.move_ttbot(speed, heading)
            
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    
    # create inflated map
    mp = MapProcessor('map')
    kr = mp.rect_kernel(5,1)
    mp.inflate_map(kr,True)
    mp.get_graph_from_map()             
    mp.print_graph_nodes()

    nav = Navigation(mp, node_name='Navigation')

    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()