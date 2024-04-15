import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from adk_node.msg import WaypointPath
from adk_node.msg import TargetPerception

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from math import sin,cos,pi
import numpy as np
from tf_transformations import quaternion_from_euler 
from tf_transformations import euler_from_quaternion 
from tf_transformations import euler_from_matrix

from utils.utility_functions import *
from mission import *
from graph import Graph

from shapely.geometry import Polygon, Point, LineString
from shapely.ops import nearest_points
from shapely import distance

# Run the labyrinth navigation experiment.
''' Conventions

Directions:
----------
0 = right
1 = down
2 = left
3 = up

Actions:
--------
left = 0
right = 1
forward = 2
'''


# %%
import os, sys
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"
sys.path.append('..')

from Environments.minigrid_labyrinth import Maze
import numpy as np
from Controllers.minigrid_controller import MiniGridController
from Controllers.meta_controller import MetaController
import pickle
import os, sys
from datetime import datetime
from MDP.high_level_mdp import HLMDP
from utils.results_saver import Results
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('sparse_waypoint_publisher')
        # ROS publishers and subscribers
        qos_profile = QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL) # Require special QOS for this message
        self.publisher_ = self.create_publisher(WaypointPath, 'adk_node/input/waypoints', qos_profile) # Waypoint topic for ROS2 API controller
        self.create_subscription(TargetPerception, 'adk_node/ground_truth/perception', self.gt_perception_callback, 10) # Ground truth perception topic
        self.create_subscription(Odometry, 'adk_node/SimpleFlight/odom_local_ned', self.odom_callback, 10) # Position topic (NED frame)
        self.odom_msg = None
        self.detected_entity_ids = [] # Keeps track of detected EOIs
        self.visited_cell_idxs = [] # Keeps track of visited cells

        # Load mission files
        self.mission = Mission('../../../mission_briefing/description2.json', '../../../mission_briefing/config.json')

        # Wait for the first ROS position message to arrive 
        while self.odom_msg == None:
            rclpy.spin_once(self)
        
        self.get_logger().info('Odom Message Received...')

        # Setup
        self.setup()
        
        # Start appropriate mission
        if self.mission.mission_class == "Area Search":
            self.run_area_search_mission()
            if not self.all_cars_detected():
                self.run_special_aoi_search()

        elif self.mission.mission_class == "Route Search":
            self.run_area_search_mission() # Change to route search function

    def all_cars_detected(self):
        for car in self.mission.car_list:
            if car.id not in self.detected_entity_ids:
                return False
        return True

    def run_special_aoi_search(self):
        AOI_points = self.mission.getSpecialAOIPoints()
        current_state = (self.n_airsim, self.e_airsim, 0)
        for point in AOI_points:
            airsim_state_list = []
            AOI_point_entry = self.mission.getSpecialAOIPointEntry(point)
            if AOI_point_entry is None: continue

            airsim_state_entry = (AOI_point_entry[1], AOI_point_entry[0], 0)
            airsim_state_AOI = (point[1], point[0], 0)

            state_list = self.get_navigation_state_list(current_state, airsim_state_entry)
            airsim_state_list.extend(state_list)

            airsim_state_list.append(airsim_state_AOI)

            state_list = self.look_around(airsim_state_AOI)
            airsim_state_list.extend(state_list)

            airsim_state_list.append(airsim_state_entry)
            current_state = airsim_state_entry
            
        self.pub_waypoint(airsim_state_list)

    def get_navigation_state_list(self, source_state, target_state):
        obs_list = []
        source_minigrid_state = airsim2minigrid(source_state)
        target_minigrid_state = airsim2minigrid(target_state)
        # Obtain a list of controllers yielding shortest path to cell of interest that does not intersect with any KOZ
        hl_controller_idx_list = self.mission.graph.find_controllers_from_node_to_node(source_minigrid_state[0], source_minigrid_state[1], target_minigird_state[0], target_minigrid_state[1], True, True)
        print("\nFind controllers from coord({}, {}) to coord({}, {})".format(source_minigrid_state[0], source_minigrid_state[1], target_minigrid_state[0], target_minigrid_state[1]))
                
        for p in hl_controller_idx_list:
            print(p)
        print("\n")

        if len(hl_controller_idx_list) > 0:
            hl_controller_idx_list = hl_controller_idx_list[0]
            hl_controller_list = self.get_controller_list(hl_controller_idx_list)

            if len(hl_controller_idx_list) > 0:
                # Execute controllers in minigrid and publish waypoints
                # print("Init State: "+str(self.obs))
                for controller in hl_controller_list:
                    init = True
                    # why need init?
                    if init:
                        # print("Final State: "+str(controller.get_final_states())+"\n")
                        # print("** Using Controller **: "+str(controller.controller_ind)+"\n")
                        init = False
                    while (self.obs != controller.get_final_states()).any():
                        action,_states = controller.predict(self.obs, deterministic=True)
                        self.obs, reward, done, info = self.env.step(action)
                        # print("Action: "+str(action))
                        # print("Current State: "+str(self.obs))
                        airsim_obs = minigrid2airsim(self.obs)
                        # print("AirSim State: "+str(airsim_obs)+"\n")
                        obs_list.append(airsim_obs)

        return obs_list

    def look_around(self, state):
        state_list = []
        delta = 1

        state[0] = state[0] + delta
        state_list.append(state)

        state[1] = state[1] + delta
        state_list.append(state)

        state[0] = state[0] - delta
        state_list.append(state)

        state[1] = state[1] - delta
        state_list.append(state)

        return state_list

    def gt_perception_callback(self, gt_perception_msg):
        entity_status = "entered" if gt_perception_msg.enter_or_leave == 0 else "left"
        entity_id = gt_perception_msg.entity_id
        self.get_logger().info('Entity {} just {} {} with probability {}...'.format(entity_id, entity_status, gt_perception_msg.camera, gt_perception_msg.probability))
        if entity_id not in self.detected_entity_ids: self.detected_entity_ids.append(entity_id)

    def odom_callback(self, odom_msg):
        self.odom_msg = odom_msg
        self.n_airsim = odom_msg.pose.pose.position.x
        self.e_airsim = odom_msg.pose.pose.position.y
        print("Current AirSim State: {}".format([self.n_airsim, self.e_airsim, 0]))

    def sort_points_by_distance(self, points, source):
            #points.sort(key = lambda p: (p.x - x)**2 + (p.y - y)**2)
            points.sort(key = lambda p: (p[0] - source[0])**2 + (p[1] - source[1])**2)
            return points    

    def get_nearest_entry(self, source_point):
        n, e, _ = source_point
        source_point = [e, n]

        node_list = []
        for node in self.mission.graph.nodes:
            node_minigrid = [node.get_point().x, node.get_point().y, 0]
            n, e, _ = minigrid2airsim(node_minigrid)
            node_airsim = [e, n]
            node_list.append(node_airsim)
        
        # print("source_point:", source_point)
        # print("node_list:", node_list)

        sorted_node_list = self.sort_points_by_distance(node_list, source_point)

        points = []
        points.append(source_point)
        for node in sorted_node_list:
                intersect = False
                points.append(node)
                linestring = LineString(points)
                for zone in self.mission.keep_out_zones:
                    if linestring.intersects(zone.polygon):
                        intersect = True
                        break
                if intersect:
                    points.pop()
                else:
                    e, n = node
                    print("Nearest AirSim Entry State: ", [n, e, 0])
                    return [n, e, 0]
        return None

    def setup(self):
        
        # Load minigrid controllers
        load_folder_name = '2024-04-01_17-26-42_minigrid_labyrinth'

        base_path = os.path.abspath(os.path.curdir)
        string_ind = base_path.find('src')
        assert(string_ind >= 0)
        base_path = base_path[0:string_ind + 4]
        base_path = os.path.join(base_path, 'verifiable-compositional-rl/src/data', 'saved_controllers')
        load_dir = os.path.join(base_path, load_folder_name)

        self.controller_list = []
        for controller_dir in os.listdir(load_dir):
            controller_load_path = os.path.join(load_dir, controller_dir)
            if os.path.isdir(controller_load_path):
                controller = MiniGridController(0, load_dir=controller_load_path)
                self.controller_list.append(controller)
        
        # re-order the controllers by index in ascending order
        reordered_list = []
        for i in range(len(self.controller_list)):
            for controller in self.controller_list:
                if controller.controller_ind == i:
                    reordered_list.append(controller)
        
        self.controller_list = reordered_list
        
        print("\n\n\n")
        self.get_logger().info('Mission Begins...')
        print("\n\n\n")

        # Initialize graph structure (for computing paths) with mission and controller list data
        self.mission.graph = Graph()
        self.mission.graph.set_data(self.mission, self.controller_list)

        # Graph Tests
        # for i in range(len(self.mission.graph.keep_out_edges)):
        #     print(f"self.mission.graph.keep_out_edges[{i}] = {self.mission.graph.keep_out_edges[i]}")

        # print("\n\nTest: Find controllers from coord(12, 6) to coord(2,6)")
        # print("prune_koe = False, include_last=False")
        # for p in self.mission.graph.find_controllers_from_node_to_node(12, 6, 2, 6, False, False):
        #     print(p)

        # print("\n\nTest: Find controllers from coord(12, 6) to coord(2,6)")
        # print("prune_koe = False, include_last=True")
        # for p in self.mission.graph.find_controllers_from_node_to_node(12, 6, 2, 6, False, True):
        #     print(p)

        # print("\n\nTest: Find controllers from coord(12, 6) to coord(2,6)")
        # print("prune_koe = True, include_last=False")
        # for p in self.mission.graph.find_controllers_from_node_to_node(12, 6, 2, 6, True, False):
        #     print(p)

        # print("\n\nTest: Find controllers from coord(12, 6) to coord(2,6)")
        # print("prune_koe = True, include_last=True")
        # for p in self.mission.graph.find_controllers_from_node_to_node(12, 6, 2, 6, True, True):
        #     print(p)
            
        # exit()

        # Move to first AirSim state that corresponds to a Minigrid state
        # ------------------------------------------------------------------------------------------------

        # Get the nearest minigrid entry state of the first controller
        current_airsim_state = [self.n_airsim, self.e_airsim, 0]
        self.mission.start_airsim_state = self.get_nearest_entry(current_airsim_state)
        self.mission.start_minigrid_state = airsim2minigrid(self.mission.start_airsim_state) # Get the airsim state for the entry state of the first controller

        # Move to entry state of the first controller
        self.get_logger().info('Moving to Initial Entry Position...')
        self.pub_waypoint([self.mission.start_airsim_state])

        # Check if the drone reached entry state of the first controller
        current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
        final_minigrid_state = self.mission.start_minigrid_state
        while (current_minigrid_state != final_minigrid_state):
            rclpy.spin_once(self)
            current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
            print("Current State: {}, Final State: {}".format(current_minigrid_state, final_minigrid_state))
        
        self.get_logger().info('Reached Initial Entry Position...')


        env_settings = {
            'agent_start_states' : [tuple(self.mission.start_minigrid_state)],
            'slip_p' : 0.0,
        }

        self.env = Maze(**env_settings)
        self.obs = self.env.reset() # Get the first minigrid state

    def run_area_search_mission(self):
        # Itereate through each car in the list of cars
        for car in self.mission.car_list:
            print('Looking for car: ', car.id)
            print('Priority: ', car.priority)
            if car.id in self.detected_entity_ids: continue # Skip if we already detected the car
            self.run_single_eoi_search(car) # Planning for how to search for the car

    # Out a list of controller objects given controller id
    def get_controller_list(self,hl_controller_idx_list):
        controller_list = []
        for hl_controller_id in hl_controller_idx_list:
            for controller in self.controller_list:
                if controller.controller_ind == hl_controller_id:
                    controller_list.append(controller)
        return controller_list

    # Given a list of controllers to execute, execute each controller in minigrid and then publish waypoints to AirSim
    def run_minigrid_solver_and_pub(self,hl_controller_list):
        obs_list = []
        # print("Init State: "+str(self.obs))
        for controller in hl_controller_list:
            init = True
            if init:
                # print("Final State: "+str(controller.get_final_states())+"\n")
                # print("** Using Controller **: "+str(controller.controller_ind)+"\n")
                init = False
            while (self.obs != controller.get_final_states()).any():
                action,_states = controller.predict(self.obs, deterministic=True)
                self.obs, reward, done, info = self.env.step(action)
                # print("Action: "+str(action))
                # print("Current State: "+str(self.obs))
                airsim_obs = minigrid2airsim(self.obs)
                # print("AirSim State: "+str(airsim_obs)+"\n")
                obs_list.append(airsim_obs)
        self.pub_waypoint(obs_list)

    def traverse_koz_and_return(self, cell_idx, car):
        self.init_airsim_position = [self.n_airsim, self.e_airsim, 0]
        self.init_minigrid_state = airsim2minigrid(self.init_airsim_position)
        self.init_airsim_point = Point(self.e_airsim, self.n_airsim)
        current_airsim_point = Point(self.e_airsim, self.n_airsim)
        koz = self.mission.cells[cell_idx].keep_out_zone
        print("KOZ and current airsim point: ",koz.polygon, current_airsim_point)
        nearest_point, _ = nearest_points(koz.polygon, current_airsim_point)
        target_airsim_position = [nearest_point.y, nearest_point.x, 0]
        mid_airsim_position = [(target_airsim_position[0]+self.init_airsim_position[0])/2, (target_airsim_position[1]+self.init_airsim_position[1])/2, 0]
        
        target_airsim_position_list = [mid_airsim_position, target_airsim_position]
        print(target_airsim_position_list)
        print("Nearest AirSim Position to KOZ: "+str(nearest_point))
        self.get_logger().info('Moving to One End of KOZ...')
        self.pub_waypoint(target_airsim_position_list)

        # While moving to the nearest point to the KOZ keep track of detected entities
        while (distance(current_airsim_point, nearest_point) > 30): # TODO: if the distance is set to 15, drone gets stuck at about 25m from KOZ
            rclpy.spin_once(self)
            print("Current Distance: ", distance(current_airsim_point, nearest_point))
            current_airsim_point = Point(self.e_airsim, self.n_airsim)
            print("Current AirSim Position: "+str(current_airsim_point))
            if car.id in self.detected_entity_ids:
                break
    
        # Return back to entry of the last cell
        current_airsim_position = [current_airsim_point.y, current_airsim_point.x, 0]
        current_minigrid_state = airsim2minigrid(current_airsim_position)
        mid_airsim_position = [(current_airsim_position[0]+self.init_airsim_position[0])/2, (current_airsim_position[1]+self.init_airsim_position[1])/2, 0]
        
        target_airsim_position_list = [mid_airsim_position, self.init_airsim_position]
        self.get_logger().info('Returning to Initial AirSim Position...')
        self.pub_waypoint(target_airsim_position_list)
        print("Moving back to initial position: "+str(self.init_airsim_position))
        while ((current_minigrid_state != self.init_minigrid_state)):
            rclpy.spin_once(self)
            current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
            print("Current Minigrid State: {}, Final State: {}".format(current_minigrid_state, self.init_minigrid_state))
        
        if car.id in self.detected_entity_ids:
            return True
        
        return False

    def run_single_eoi_search(self, car):
        # Iterate through each region in AOI
        for region in car.map:
            print('Probability: ',region.probability)
            print('Polygon: ', region.polygon)
            print('Cells: ', region.cells)
            # Iterate through each cell that covers the region -> Plan how to get to one of the destination the cell states from current state
            for cell_idx in region.cells:
                if cell_idx in self.visited_cell_idxs: continue # Skip if we already visited the cell
                
                current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))

                # Obtain a list of controllers yielding shortest path to cell of interest that does not intersect with any KOZ
                hl_controller_idx_list = self.mission.graph.find_controllers_from_node_to_edge(current_minigrid_state[0], current_minigrid_state[1], cell_idx, True, True)
                print("\nFind controllers from coord({}, {}) to cell {}".format(current_minigrid_state[0], current_minigrid_state[1], cell_idx))
                
                for p in hl_controller_idx_list:
                    print(p)
                print("\n")

                # Move to a cell based on car priority and probability
                # ------------------------------------------------------------------------------------------------
        
                if len(hl_controller_idx_list) == 0: continue # Skip if we have no path obtained
                hl_controller_idx_list = hl_controller_idx_list[0]
                hl_controller_list = self.get_controller_list(hl_controller_idx_list)
                
                # Only go upto the penultimate cell if a KOZ is in the last cell
                if self.mission.cells[cell_idx].in_keep_out_zone:
                    print("Cell {} is in KOZ and controller list is: {}".format(cell_idx, hl_controller_idx_list))
                    target_controller_idx = hl_controller_idx_list[-1]
                    target_controller = hl_controller_list[-1]
                    hl_controller_idx_list.pop()
                    hl_controller_list.pop()

                # Do not visit the same cell again
                for controller_idx in hl_controller_idx_list:
                    cell_id = math.floor(controller_idx/2)
                    if cell_id not in self.visited_cell_idxs:
                        self.visited_cell_idxs.append(cell_id)
                        self.mission.cells[cell_id].visited = True
                print("Visited Cells:", self.visited_cell_idxs)
                
                if len(hl_controller_idx_list) != 0:
                    self.get_logger().info('Moving to Next Cell...')
                    # Execute controllers in minigrid and publish waypoints
                    self.run_minigrid_solver_and_pub(hl_controller_list)
                    final_minigrid_state = list(hl_controller_list[-1].get_final_states()[0])
                    
                    # While moving to the waypoints keep track of detected entities
                    while (current_minigrid_state != final_minigrid_state):
                        rclpy.spin_once(self)
                        current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
                        print("Current Minigrid State: {}, Final State: {}".format(current_minigrid_state, final_minigrid_state))   
                    self.get_logger().info('Reached Target Cell...')
                    
                    if car.id in self.detected_entity_ids:
                        return

                # Last cell in KOZ handling
                # ------------------------------------------------------------------------------------------------
                
                # Once at entry to the target cell, check which controller has KOZ
                if self.mission.cells[cell_idx].in_keep_out_zone:
                    ret = self.traverse_koz_and_return(cell_idx, car)
                    if ret == True: return

                    self.get_logger().info('Could not find target car; moving to other end of KOZ...')
                    
                    # Done traversing one end of the cell with KOZ
                    # ------------------------------------------------------------------------------------------------
                    
                    other_end_minigrid_state = list(target_controller.get_final_states()[0])
                    hl_controller_idx_list = self.mission.graph.find_controllers_from_node_to_node(current_minigrid_state[0], current_minigrid_state[1], other_end_minigrid_state[0], other_end_minigrid_state[1], True, False)
                    print("\nFind controllers from coord({}, {}) to coord({}, {})".format(current_minigrid_state[0], current_minigrid_state[1], other_end_minigrid_state[0], other_end_minigrid_state[1]))
                    
                    for p in hl_controller_idx_list:
                        print(p)
                    print("\n")

                    # Moving to the other end of KOZ
                    # ------------------------------------------------------------------------------------------------
                    if len(hl_controller_idx_list) == 0: continue # Skip if we have no path obtained
                    hl_controller_idx_list = hl_controller_idx_list[0]
                    hl_controller_list = self.get_controller_list(hl_controller_idx_list)

                    if len(hl_controller_idx_list) != 0:
                        self.get_logger().info('Moving to Next Cell...')
                        # Execute controllers in minigrid and publish waypoints
                        self.run_minigrid_solver_and_pub(hl_controller_list)
                        final_minigrid_state = list(hl_controller_list[-1].get_final_states()[0])
                        
                        # While moving to the waypoints keep track of detetcted entities
                        while (current_minigrid_state != final_minigrid_state):
                            rclpy.spin_once(self)
                            current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
                            print("Current Minigrid State: {}, Final State: {}".format(current_minigrid_state, final_minigrid_state))   
                        self.get_logger().info('Reached Target Cell...')
                        
                        if car.id in self.detected_entity_ids:
                            return
                    
                    # Last cell in KOZ handling
                    # ------------------------------------------------------------------------------------------------ 
                    
                    # Once at entry to the target cell, check which controller has KOZ
                    ret = self.traverse_koz_and_return(cell_idx, car)
                    if ret == False:
                        self.get_logger().info('Could not find target car {}'.format(car.id))

                
    def pub_waypoint(self, obs_list):
        waypoint_msg = WaypointPath()
        
        pose_msg_array = []

        for airsim_obs in obs_list:
            z = -10.0
            n_airsim, e_airsim, yaw_airsim = airsim_obs

            roll = 0
            pitch = 0
            yaw = 0

            q=quaternion_from_euler(roll*pi/180, pitch*pi/180, yaw*pi/180)


            pose_msg = PoseStamped()

            h = Header()
            h.stamp = self.get_clock().now().to_msg()

            pose_msg.header = h
            pose_msg.header.frame_id = "world_ned"
            pose_msg.pose.position.x = n_airsim
            pose_msg.pose.position.y = e_airsim
            pose_msg.pose.position.z = z

            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]

            pose_msg_array.append(pose_msg)

        waypoint_msg.path = pose_msg_array
        waypoint_msg.velocity = 5.0
        waypoint_msg.lookahead = -1.0
        waypoint_msg.adaptive_lookahead = 0.0
        # waypoint_msg.drive_train_type = "ForwardOnly"
        waypoint_msg.wait_on_last_task = False

        self.publisher_.publish(waypoint_msg)
        self.get_logger().info('Publishing Waypoints...')

    # Dummy function: returns nearest minigrid state given airsim state
    def dummy_nearest_minigrid_state(self, airsim_state):
        minigrid_state = airsim2minigrid(airsim_state)
        minigrid_state[2] = 0
        # Enter Junaid's code here
        return [22,2,0] # Change this

    def dummy_hl_controller(self, cell_idx): # add input start intersection (entry or exit condition)
        controller_list = []
        if cell_idx == 0:
            controller_list.extend([3,1])
        if cell_idx == 1:
            controller_list.extend([0,2])
        if cell_idx == 2:
            controller_list.extend([0,4])
        if cell_idx == 3:
            controller_list.append(7)
        if cell_idx == 5:
            controller_list.extend([8,12,11])
        if cell_idx == 6:
            controller_list.extend([10,13])
        if cell_idx == 7:
            controller_list.extend([10,14])
        if cell_idx == 9:
            controller_list.extend([15,18])
        if cell_idx == 10:
            controller_list.extend([22,21])
        if cell_idx == 14:
            controller_list.append(29)
        return controller_list


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
