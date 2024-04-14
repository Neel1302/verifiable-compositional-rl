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
        self.mission = Mission('../../../mission_briefing/description.json', '../../../mission_briefing/config.json')

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

            airsim_state_entry = minigrid2airsim(AOI_point_entry)
            airsim_state_AOI = minigrid2airsim(point)

            state_list = self.get_navigation_state_list(current_state, airsim_state_entry)
            airsim_state_list.extend(state_list)

            airsim_state_list.append(airsim_state_AOI)

            state_list = self.look_around(airsim_state_AOI)
            airsim_state_list.extend(state_list)

            airsim_state_list.append(airsim_state_entry)
            current_state = airsim_state_entry
            
        self.pub_waypoint(airsim_state_list)

    def get_navigation_state_list(self, source_state, target_state):
        state_list = []
        # TODO
        state_list.append(source_state)
        state_list.append(target_state)

        return state_list

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

    # Dummy function: returns nearest minigrid state given airsim state
    def dummy_nearest_minigrid_state(self, airsim_state):
        minigrid_state = airsim2minigrid(airsim_state)
        minigrid_state[2] = 0
        # Enter Junaid's code here
        return [22,2,0] # Change this

    def setup(self):
        # Get the minigrid entry state of the first controller
        # TODO: can't invoke nearest_minigrid_state() before initializing graph structure
        self.mission.start_minigrid_state = self.dummy_nearest_minigrid_state(self.mission.start_airsim_state) # Replace with Junaid's code here
        self.mission.start_airsim_state = minigrid2airsim(self.mission.start_minigrid_state) # Get the airsim state for the entry state of the first controller

        # Move to entry state of the first controller
        self.pub_waypoint([self.mission.start_airsim_state])

        # Check if the drone reached entry state of the first controller
        current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
        final_minigrid_state = self.mission.start_minigrid_state
        while (current_minigrid_state != final_minigrid_state):
            rclpy.spin_once(self)
            current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
            print("Current State: {}, Final State: {}".format(current_minigrid_state, final_minigrid_state))

        # Minigrid controller setup
        env_settings = {
            'agent_start_states' : [tuple(self.mission.start_minigrid_state)],
            'slip_p' : 0.0,
        }

        self.env = Maze(**env_settings)

        num_rollouts = 5
        meta_controller_n_steps_per_rollout = 500

        # %% Set the load directory (if loading pre-trained sub-systems) or create a new directory in which to save results

        load_folder_name = '2024-04-01_17-26-42_minigrid_labyrinth'

        base_path = os.path.abspath(os.path.curdir)
        string_ind = base_path.find('src')
        assert(string_ind >= 0)
        base_path = base_path[0:string_ind + 4]
        base_path = os.path.join(base_path, 'verifiable-compositional-rl/src/data', 'saved_controllers')
        load_dir = os.path.join(base_path, load_folder_name)

        # %% Load the sub-system controllers
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

        # Initialize graph structure (for computing paths) with mission and controller list data
        self.mission.graph = Graph()
        self.mission.graph.set_data(self.mission, self.controller_list)

        a_graph = self.mission.graph
        # # Test: find all paths in the graph
        # print("\n\nTest: find all paths in the graph")
        # id_to_paths = a_graph.find_all_paths()
        # for node_id, paths in id_to_paths.items():
        #     print(node_id + ":", len(paths))
        #     for path, mh in paths:
        #         print("\t", mh, ":", path)
    
        # # Test: find all paths from one vertex to another
        # print("\n\nTest: find all paths from one vertex to another")
        # sorted_paths = a_graph.find_paths('0', '1')
        # print(f"\nPaths from 0 to 1: {len(sorted_paths)}")
        # for path, mh in sorted_paths:
        #     print("\t", mh, ":", path)
        # sorted_paths = a_graph.find_paths('0', '2')
        # print(f"\nPaths from 0 to 2: {len(sorted_paths)}")
        # for path, mh in sorted_paths:
        #     print("\t", mh, ":", path)

        # # Test: find all paths from a vertex to a cell
        # print("\n\nTest: find all paths from a vertex to a cell")
        # sorted_paths = a_graph.find_paths_to_cell('0', '1', '2')
        # print(f"\nPaths from 0 ending with 1,2 or 2,1: {len(sorted_paths)}")
        # for path, mh in sorted_paths:
        #     print("\t", mh, ":", path)

        # # Test: find the nearest vertex in the graph to the input coordinates
        # # print("Enter point x y to check for nearest intersection: ", end="")
        # # x, y = map(int, input().split())
        # print("\n\nTest: find the nearest vertex in the graph to the input coordinates")
        # x, y = 10, 10
        # p = Point(x, y)
        # nearest_node = a_graph.locate_nearest_node(p)
        # print(f"Nearest node to ({x},{y}) is ", end="")
        # nearest_node.print()
        # print()

        # # Test: find paths between two arbitrary points after finding nodes closest to them
        # print("\n\nTest: find paths between two arbitrary points after finding nodes closest to them")
        # p1 = Point(10, 10)
        # n1 = a_graph.locate_nearest_node(p1)
        # p2 = Point(20, 20)
        # n2 = a_graph.locate_nearest_node(p2)
        # n1.print()
        # n2.print()

        # paths = a_graph.find_paths(n1.get_id(), n2.get_id())
        # for p in paths:
        #     print(p)
        
        # # Test: find neighbors of an arbitrary node
        # print("\n\nTest: find neighbors of an arbitrary node")
        # n1.print()
        # neighbors = a_graph.get_neighboring_nodes(n1.get_id())
        # print(neighbors)

        # # Test: find all paths between a pair of neighboring nodes
        # print("\n\nTest: find all paths between a pair of neighboring nodes")
        # n1.print()
        # n2 = a_graph.get_node(neighbors[0])
        # n2.print()
        # paths = a_graph.find_paths(n1.get_id(), n2.get_id())
        # print(paths)
        # for p in paths:
        #     print(p)

        # # Test: Convert paths of nodes to a path of controllers
        # print("\n\nTest: Convert paths of nodes to a path of controllers")
        # for p in a_graph.convert_to_controllers(paths, True, True):
        #     print(p)
        
        # print("\n\nTest: Adding controllers 16 and 17 to the list of keep out edges")
        # a_graph.keep_out_edges[16]=True
        # a_graph.keep_out_edges[17]=True
            
        # print("\n\nTest: Paths after pruning paths with keep out edges")
        # for p in a_graph.convert_to_controllers(paths, True, True):
        #     print(p)
        
        # print("\n\nTest: Removing controllers 16 and 17 to the list of keep out edges")
        # a_graph.keep_out_edges[16]=False
        # a_graph.keep_out_edges[17]=False
            
        # # Test: find all paths from a vertex to a cell
        # print("\n\nTest: find all paths from a vertex to a cell")
        # print(f"\nPaths from 0 ending with 1,2 or 2,1: {len(sorted_paths)}")
        # for p in a_graph.convert_to_controllers(sorted_paths, True, True):
        #     print(p)

        # print("\n\nTest: Adding controllers 6, 7, 16 and 17 to the list of keep out edges")
        # a_graph.keep_out_edges[16]=True
        # a_graph.keep_out_edges[17]=True
        # a_graph.keep_out_edges[6]=True
        # a_graph.keep_out_edges[7]=True
            
        # print("\n\nTest: Paths after pruning paths with keep out edges")
        # for p in a_graph.convert_to_controllers(sorted_paths, True, True):
        #     print(p)
        
        # print("\n\nTest: Removing keep out edges")
        # a_graph.keep_out_edges[16]=False
        # a_graph.keep_out_edges[17]=False
        # a_graph.keep_out_edges[6]=False
        # a_graph.keep_out_edges[7]=False
            
        # print("\n\nTest: Find controllers from coord(12, 6) to cell 9")
        # for p in a_graph.find_controllers_from_node_to_edge(12, 6, 9, False, True):
        #     print(p)

        # for p in a_graph.find_controllers_from_node_to_edge(12, 6, 9, True, True):
        #     print(p)

        print("\n\nTest: Find controllers from coord(12, 6) to coord(12, 12)")
        for p in a_graph.find_controllers_from_node_to_node(12, 6, 12, 12, False, True):
            print(p)

        print("\n\nTest: Find controllers from coord(12, 6) to coord(12, 12)")
        for p in a_graph.find_controllers_from_node_to_node(12, 6, 12, 12, True, True):
            print(p)

        # print("\n\nTest: Adding controllers 6, 7, 16 and 17 to the list of keep out edges")
        # a_graph.keep_out_edges[16]=True
        # a_graph.keep_out_edges[17]=True
        # a_graph.keep_out_edges[6]=True
        # a_graph.keep_out_edges[7]=True

        # print("\n\nTest: Find controllers from coord(2, 2) to cell 12")
        # for p in a_graph.find_controllers_from_node_to_edge(2, 2, 12, True, True):
        #     print(p)

        exit()

                
        self.obs = self.env.reset() # Get the first minigrid state

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
        print("Init State: "+str(self.obs))
        for controller in hl_controller_list:
            init = True
            if init:
                print("Final State: "+str(controller.get_final_states())+"\n")
                print("** Using Controller **: "+str(controller.controller_ind)+"\n")
                init = False
            while (self.obs != controller.get_final_states()).any():
                action,_states = controller.predict(self.obs, deterministic=True)
                self.obs, reward, done, info = self.env.step(action)
                print("Action: "+str(action))
                print("Current State: "+str(self.obs))
                airsim_obs = minigrid2airsim(self.obs)
                print("AirSim State: "+str(airsim_obs)+"\n")
                obs_list.append(airsim_obs)
        self.pub_waypoint(obs_list)

    def run_single_eoi_search(self, car):
        # Iterate through each region in AOI
        for region in car.map:
            print('Probability: ',region.probability)
            print('Polygon: ', region.polygon)
            print('Cells: ', region.cells)
            # Iterate through each cell that covers the region -> Plan how to get to one of the destination the cell states from current state
            for cell_idx in region.cells:
                if cell_idx in self.visited_cell_idxs: continue # Skip if we already visited the cell
                
                # Obtain a list of controllers yielding shortest path to cell of interest that does not intersect with any KOZ
                hl_controller_idx_list = self.dummy_hl_controller(cell_idx) # Replace with Junaid's code
                hl_controller_list = self.get_controller_list(hl_controller_idx_list)
                if len(hl_controller_idx_list) == 0: continue # Skip if we have no path obtained
                
                # Do not visit the same cell again
                for controller_idx in hl_controller_idx_list:
                    cell_id = math.floor(controller_idx/2)
                    if cell_id not in self.visited_cell_idxs:
                        self.visited_cell_idxs.append(cell_id)
                        self.mission.cells[cell_id].visited = True
                print("Visited Cells:", self.visited_cell_idxs)
                
                # Only go upto the penultimate cell if a KOZ is in the last cell
                if self.mission.cells[cell_idx].in_keep_out_zone:
                    print("Cell {} is in KOZ and controller list is: {}".format(cell_idx, hl_controller_list))
                    target_controller_idx = hl_controller_idx_list[-1]
                    target_controller = hl_controller_list[-1]
                    hl_controller_idx_list.pop()
                    hl_controller_list.pop()
                
                if len(hl_controller_idx_list) != 0:
                    # Execute controllers in minigrid and publish waypoints
                    self.run_minigrid_solver_and_pub(hl_controller_list)
                    current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
                    final_minigrid_state = list(hl_controller_list[-1].get_final_states()[0])
                    
                    # While moving to the waypoints keep track of detetcted entities
                    while (current_minigrid_state != final_minigrid_state):
                        rclpy.spin_once(self)
                        current_minigrid_state = airsim2minigrid((self.n_airsim, self.e_airsim, 0))
                        print("Current State: {}, Final State: {}".format(current_minigrid_state, final_minigrid_state))   
                        if car.id in self.detected_entity_ids:
                            # self.pub_waypoint([self.mission.start_airsim_state]) # Need to think about and implement how we will get back to prevoius state.
                            return
                
                if self.mission.cells[cell_idx].in_keep_out_zone:            
                    # Once at entry to the target cell, check which controller has
                    self.init_airsim_position = [self.n_airsim, self.e_airsim, 0]
                    self.init_airsim_point = Point(self.n_airsim, self.e_airsim)
                    curr_airsim_point = Point(self.n_airsim, self.e_airsim)
                    koz = self.mission.cells[cell_idx].keep_out_zone
                    print("TEST: ",koz.polygon, curr_airsim_point)
                    nearest_point, _ = nearest_points(koz.polygon, curr_airsim_point)
                    target_airsim_position = [nearest_point.x, nearest_point.y, 0]
                    self.pub_waypoint([target_airsim_position])

                    # While moving to the nearest point to the KOZ keep track of detetcted entities
                    print("Nearest AirSim Position to KOZ: "+str(nearest_point))
                    while (distance(curr_airsim_point, nearest_point) > 15):
                        rclpy.spin_once(self)
                        curr_airsim_point = Point(self.n_airsim, self.e_airsim)
                        print("Current AirSim Position: "+str(curr_airsim_point))
                        if car.id in self.detected_entity_ids:
                            break
                    
                    # Return back to entry of the last cell
                    self.pub_waypoint([self.init_airsim_position])
                    print("Moving back to initial position: "+str(self.init_airsim_position))
                    while (curr_airsim_point != self.init_airsim_point):
                        rclpy.spin_once(self)
                        curr_airsim_point = Point(self.n_airsim, self.e_airsim)
                        print("Current AirSim Position: "+str(curr_airsim_point))

                
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
