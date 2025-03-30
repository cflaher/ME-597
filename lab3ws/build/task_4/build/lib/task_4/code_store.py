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
    
    # Do not edit below (required for autograder)
    self.astarTime = Float32()
    self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9-self.start_time)
    self.calc_time_pub.publish(self.astarTime)

    def reconstruct_path(self, start_node_name, end_node_name):
        """Reconstructs the shortest path from start node to end node.
        @param  start_node_name  Name of the start node.
        @param  end_node_name	Name of the end node.
        @return path, dist   	List of node names forming the path and total distance.
        """

        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return list(reversed(total_path))
    
    # Get final path
    if end_node_name in came_from or end_node_name == start_node_name:
        node_path = reconstruct_path(end_node_name)
    else:
        self.get_logger().error("No path found")
        return path
    
    # Convert to ROS Path message
    for node_name in node_path:
        x, y = map(int, node_name.split(','))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x * self.resolution + self.offset_x
        pose.pose.position.y = y * self.resolution + self.offset_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    
    # Timing and logging
    self.astarTime = Float32()
    self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9 - self.start_time)
    self.calc_time_pub.publish(self.astarTime)
    
    return path

################################################################################

def reconstruct_path(self, start_node_name, end_node_name):
            """Reconstructs the shortest path from start node to end node.
            @param  start_node_name  Name of the start node.
            @param  end_node_name	Name of the end node.
            @return path, dist   	List of node names forming the path and total distance.
            """

            total_path = [current]
            while current in came_from:
                current = came_from[current]
                total_path.append(current)
            return list(reversed(total_path))
        
        # Get final path
        if end_node_name in came_from or end_node_name == start_node_name:
            node_path = reconstruct_path(start_node_name, end_node_name)
        else:
            self.get_logger().error("No path found")
            return path

################################################################################

 def path_follower(self, vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        speed = 0.0
        heading = 0.0
        
        # TODO: IMPLEMENT PATH FOLLOWER

#       speed controller       

        # Time period (should be defined as class variable or parameter)
        self.timer_period = 0.1  # 10Hz operation
        
        # PID parameters
        kp_dist = 0.5
        ki_dist = 0.0001
        kd_dist = 0.15
        
        kp_heading = 0.5
        ki_heading = 0.0001
        kd_heading = 0.15
        
        # Distance control
        dx = current_goal_pose.pose.position.x - vehicle_pose.pose.position.x
        dy = current_goal_pose.pose.position.y - vehicle_pose.pose.position.y
        distance = np.sqrt(dx*dx + dy*dy)
        
        # Calculate heading to target
        target_heading = math.atan2(dy, dx)
        
        # Current vehicle heading
        q = vehicle_pose.pose.orientation
        current_yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
        
        # Distance error
        self.dist_error = distance
        self.dist_integral_error += self.dist_error * self.timer_period
        self.dist_integral_error = max(min(self.dist_integral_error, 0.1), -0.1)  # Anti-windup
        
        if not hasattr(self, 'prev_dist_error'):
            self.prev_dist_error = 0.0
        
        self.dist_derivative_error = (self.dist_error - self.prev_dist_error) / self.timer_period
        self.prev_dist_error = self.dist_error
        
        speed = kp_dist * self.dist_error + ki_dist * self.dist_integral_error + kd_dist * self.dist_derivative_error
        
        # Cap speed
        speed = min(speed, 0.2)
        
        # Heading error (normalize to [-pi, pi])
        heading_error = target_heading - current_yaw
        while heading_error > math.pi:
            heading_error -= 2*math.pi
        while heading_error < -math.pi:
            heading_error += 2*math.pi
        
        self.heading_error = heading_error
        self.heading_integral_error += self.heading_error * self.timer_period
        self.heading_integral_error = max(min(self.heading_integral_error, 0.1), -0.1)  # Anti-windup
        
        if not hasattr(self, 'prev_heading_error'):
            self.prev_heading_error = 0.0
        
        self.heading_derivative_error = (self.heading_error - self.prev_heading_error) / self.timer_period
        self.prev_heading_error = self.heading_error
        
        heading = kp_heading * self.heading_error + ki_heading * self.heading_integral_error + kd_heading * self.heading_derivative_error
        
        # Cap turning rate
        heading = max(min(heading, 0.5), -0.5)
        
        return speed, heading

################################################################################

    def find_nearest_valid_node(self, target_node_name):
        """Find the nearest valid node in the graph to the target node"""
        target_x, target_y = map(int, target_node_name.split(','))
        
        # Scan nodes in increasing radius
        for radius in range(10):  # Adjust radius as needed
            for dx in range(-radius, radius+1):
                for dy in range(-radius, radius+1):
                    if abs(dx) + abs(dy) == radius:  # Manhattan distance
                        candidate_node_name = f"{target_x + dx},{target_y + dy}"
                        if candidate_node_name in self.graph.g:
                            self.get_logger().info(f"Nearest node to {target_node_name} is {candidate_node_name}")
                            return candidate_node_name
        
        self.get_logger().error(f"No valid node found near {target_node_name}")
        return None

################################################################################

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

        ############################################################################

    def get_path_idx(self, path, vehicle_pose):
        """Find the next waypoint to follow in the path."""
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position in the path pointing to the next goal pose to follow.
        """
        if not path.poses:
            return 0
        
        # Find the closest point that's at least 1 meter ahead
        min_distance = float('inf')
        target_idx = 0
        
        for i, pose in enumerate(path.poses):
            # Calculate distance to this waypoint
            dx = pose.pose.position.x - vehicle_pose.pose.position.x
            dy = pose.pose.position.y - vehicle_pose.pose.position.y
            distance = np.sqrt(dx*dx + dy*dy)
            
            # If within reasonable distance and closer than what we've found so far
            if distance >= 0.5 and distance < min_distance:
                min_distance = distance
                target_idx = i
################################################################################

    def path_follower(self, vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path.
        @return speed, heading         Tuple containing linear speed and angular heading commands
        """
        # Initialize PID variables if they don't exist
        if not hasattr(self, 'prev_dist_error'):
            self.prev_dist_error = 0.0
            self.dist_integral_error = 0.0
            
        if not hasattr(self, 'prev_heading_error'):
            self.prev_heading_error = 0.0
            self.heading_integral_error = 0.0

        # PID parameters - tune these for your robot
        kp_dist = 0.3      # Proportional gain for distance
        ki_dist = 0.001    # Integral gain for distance
        kd_dist = 0.1      # Derivative gain for distance
        
        kp_heading = 0.8   # Proportional gain for heading
        ki_heading = 0.001 # Integral gain for heading
        kd_heading = 0.2   # Derivative gain for heading

        # Calculate distance to goal
        dx = current_goal_pose.pose.position.x - vehicle_pose.pose.position.x
        dy = current_goal_pose.pose.position.y - vehicle_pose.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate target heading (angle to goal in world frame)
        target_heading = math.atan2(dy, dx)

        # Get current vehicle yaw from quaternion
        q = vehicle_pose.pose.orientation
        # Convert quaternion to Euler angles (yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Calculate heading error (difference between target and current)
        heading_error = target_heading - current_yaw
        
        # Normalize heading error to [-π, π]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Distance PID controller
        self.dist_error = distance
        self.dist_integral_error += self.dist_error * self.timer_period
        # Anti-windup for integral term
        self.dist_integral_error = max(min(self.dist_integral_error, 0.5), -0.5)
        self.dist_derivative_error = (self.dist_error - self.prev_dist_error) / self.timer_period
        self.prev_dist_error = self.dist_error
        
        speed = (kp_dist * self.dist_error + 
                ki_dist * self.dist_integral_error + 
                kd_dist * self.dist_derivative_error)

        # Heading PID controller
        self.heading_error = heading_error
        self.heading_integral_error += self.heading_error * self.timer_period
        # Anti-windup for integral term
        self.heading_integral_error = max(min(self.heading_integral_error, 0.5), -0.5)
        self.heading_derivative_error = (self.heading_error - self.prev_heading_error) / self.timer_period
        self.prev_heading_error = self.heading_error
        
        heading = (kp_heading * self.heading_error + 
                ki_heading * self.heading_integral_error + 
                kd_heading * self.heading_derivative_error)

        # Apply speed and heading limits
        max_speed = 0.2      # m/s
        max_heading = 0.8    # rad/s
        
        speed = max(min(speed, max_speed), -max_speed)
        heading = max(min(heading, max_heading), -max_heading)

        # If we're very close to the goal, stop
        if distance < 0.1:  # 10 cm threshold
            speed = 0.0
            heading = 0.0

        # Debug logging
        self.get_logger().debug(f"Distance: {distance:.2f}m, Speed: {speed:.2f}m/s")
        self.get_logger().debug(f"Heading error: {math.degrees(heading_error):.1f}°, Turn rate: {heading:.2f}rad/s")

        return speed, heading

################################################################################

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
        
        if math.degrees(heading_error) > 10.0:
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

################################################################################

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
            rclpy.spin_once(self, timeout_sec=0.3)
            
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

            self.get_logger().info('run 1')

            # Follow path if we have one
            if has_goal and len(path.poses) > 0:
                idx = self.get_path_idx(path, self.ttbot_pose)
                
                self.get_logger().info('run 2')

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
                
                self.get_logger().info('run 3')

                # Follow the path
                #current_goal = path.poses[idx]
                current_goal = self.goal_pose
                self.get_logger().info(f"Current goal: {current_goal.pose.position}")
                speed, heading = self.path_follower(self.ttbot_pose, current_goal)
                self.move_ttbot(speed, heading)
            
                self.get_logger().info('run 4')

            self.rate.sleep()

            self.get_logger().info('run 5')

            #######

            def timer_callback(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        # while rclpy.ok():
        #     # Call the spin_once to handle callbacks
        #     rclpy.spin_once(self, timeout_sec=0.3)  # Process callbacks without blocking

        #     # 1. Create the path to follow
        #     path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
        #     # 2. Loop through the path and move the robot
        #     idx = self.get_path_idx(path, self.ttbot_pose)
        #     current_goal = path.poses[idx]
        #     speed, heading = self.path_follower(self.ttbot_pose, current_goal)
        #     self.move_ttbot(speed, heading)

        #     self.rate.sleep()
        #     # Sleep for the rate to control loop timing

        self.get_logger().info('Timer callback executed')

        # # 1. Create the path to follow
        # path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
        # # 2. Loop through the path and move the robot
        # idx = self.get_path_idx(path, self.ttbot_pose)
        # current_goal = path.poses[idx]
        # speed, heading = self.path_follower(self.ttbot_pose, current_goal)    

        # Check for non-zero coordinates
        if (self.ttbot_pose.pose.position.x == 0 and 
            self.ttbot_pose.pose.position.y == 0 and 
            self.goal_pose.pose.position.x == 0 and 
            self.goal_pose.pose.position.y == 0):
            self.get_logger().warn('Received zero coordinates. Waiting for valid poses...')
            return    
        
        # If we receive a new goal, plan a new path
        if not has_goal and self.goal_pose.header.stamp != rclpy.time.Time():
            self.get_logger().info('New goal received, planning path...')
            path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            self.path_pub.publish(path)
            has_goal = True

        idx = self.get_path_idx(path, self.ttbot_pose)
        current_goal = path.poses[idx]
        speed, heading = self.path_follower(self.ttbot_pose, current_goal)

        # Check if we reached the final goal
        if idx == len(path.poses) - 1:
            self.get_logger().info('Last index')
            # Check if we're close enough to the final goal
            dx = path.poses[-1].pose.position.x - self.ttbot_pose.pose.position.x
            dy = path.poses[-1].pose.position.y - self.ttbot_pose.pose.position.y
            if np.sqrt(dx*dx + dy*dy) < 0.1:  # 10cm tolerance
                self.get_logger().info('Goal reached!')
                self.move_ttbot(0.0, 0.0)  # Stop the robot
                has_goal = False
        else:
            self.move_ttbot(speed, heading)
            self.get_logger().info('Following path...')