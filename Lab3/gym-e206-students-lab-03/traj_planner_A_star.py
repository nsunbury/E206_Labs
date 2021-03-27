# E206 Motion Planning

# Simple planner
# C Clark

import math
import dubins
import random
import matplotlib.pyplot as plt
from traj_planner_utils import *
import numpy as np
import copy

class Node():

  def __init__(self, state, parent_node, g_cost, h_cost):
    self.state = state
    self.parent_node = parent_node
    self.g_cost = g_cost
    self.h_cost = h_cost
    self.f_cost = self.g_cost + self.h_cost

  def getState(self):
    return self.state

  def get_total_cost(self):
    return self.f_cost

  def manhattan_distance_to_node(self, node):
    return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])

  def manhattan_distance_to_state(self, state):
    return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])

  def euclidean_distance_to_state(self, state):
    return math.sqrt( (self.state[1] - state[1])**2 + (self.state[2] - state[2])**2 )

class A_Star_Planner():

  DIST_TO_GOAL_THRESHOLD = 0.5 #m
  CHILDREN_DELTAS = [-0.5, -0.25, 0.0, 0.25, 0.5]
  # CHILDREN_DELTAS = [-2.5, -1.5, -0.5, -0.25, 0.0, 0.25, 0.5, 1.5, 2.5]
  DISTANCE_DELTA = 1.5 #m
  EDGE_TIME = 5 #s
  LARGE_NUMBER = 9999999

  def __init__(self):
    self.fringe = []

  def construct_traj(self, initial_state, desired_state, objects, walls):
    """ Construct a trajectory in the X-Y space and in the time,X,Y,Theta space.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
    """
    self.fringe = []
    self.desired_state = desired_state
    self.objects = objects
    self.walls = walls
    '''
    - Create inital node
    - Loop until goal is found
    - tie? (check g-cost)
    - if goal is found return path (how do we store path?)
    - else return []
    '''
    init_node = self.create_initial_node(initial_state)
    self.add_to_fringe(init_node)
    # print("------------------------")
    # TODO: maybe change the while loop to also check theta (might cause looping if we do)
    goal = self.generate_goal_node(init_node, desired_state)
    while(goal is None and len(self.fringe)!= 0 and len(self.fringe)< 2000):
      best_node = self.get_best_node_on_fringe()
      goal = self.generate_goal_node(best_node, desired_state)

      #Distance 1398m, 96plots: 
      children= []
      if(best_node.parent_node == None):
        print("init node")
        children = self.get_children_with_backwards(best_node)
      else:
        children = self.get_children(best_node)

      #Distance: 1357m, 85plots 
      # children = self.get_children(best_node)

      for child in children:
        self.add_to_fringe(child)
    if(goal is None):
      print("NO VALID PATHS")
      return []

    traj = self.build_traj(goal)
    return traj[0], traj[1]

  def add_to_fringe(self, node):
    if(len(self.fringe) == 0):
      self.fringe.append(node)
    else:
      i = 0

      # Insertion sort (inserting in an ascending order so list stays sorted)
      while(i< len(self.fringe) and self.fringe[i].get_total_cost() < node.get_total_cost()):
        i+= 1
      self.fringe.insert(i,node)

  def get_best_node_on_fringe(self):
    if(len(self.fringe) == 0):
      print(f"Fringe is empty, Current fringe:{self.fringe}")
      return
    return self.fringe.pop(0)

  def get_children_with_backwards(self, node_to_expand):
    children_list = []
    parent_node = node_to_expand
    

    parent_node_backwards = copy.deepcopy(node_to_expand)
    parent_node_backwards.state[3] = angle_diff(parent_node_backwards.state[3] + np.pi)
    parent_nodes = [parent_node, parent_node_backwards]
    print(node_to_expand .state)
    print(parent_nodes[1].state)
    for child_delta in A_Star_Planner.CHILDREN_DELTAS:
      for curr_parent in parent_nodes:
        parent_node_time, parent_node_x, parent_node_y, parent_node_theta = curr_parent.state
        child_node_state = []
        child_node_state.append(parent_node_time + A_Star_Planner.EDGE_TIME)
        child_node_state.append(parent_node_x + A_Star_Planner.DISTANCE_DELTA * np.cos(parent_node_theta + child_delta))
        child_node_state.append(parent_node_y + A_Star_Planner.DISTANCE_DELTA * np.sin(parent_node_theta + child_delta))
        child_node_state.append(angle_diff(parent_node_theta + 2*child_delta))

        child_node = self.create_node(child_node_state, curr_parent)

        # Collision checking
        if(not self.collision_found(curr_parent, child_node)):
          print(child_node.state)
          children_list.append(child_node)

    return children_list


  def get_children(self, node_to_expand):
    children_list = []
    parent_node = node_to_expand
    parent_node_time, parent_node_x, parent_node_y, parent_node_theta = node_to_expand.state

    for child_delta in A_Star_Planner.CHILDREN_DELTAS:
      child_node_state = []
      child_node_state.append(parent_node_time + A_Star_Planner.EDGE_TIME)
      child_node_state.append(parent_node_x + A_Star_Planner.DISTANCE_DELTA * np.cos(parent_node_theta + child_delta))
      child_node_state.append(parent_node_y + A_Star_Planner.DISTANCE_DELTA * np.sin(parent_node_theta + child_delta))
      child_node_state.append(angle_diff(parent_node_theta + 2*child_delta))

      child_node = self.create_node(child_node_state, parent_node)

      # Collision checking
      if(not self.collision_found(parent_node, child_node)):
        children_list.append(child_node)

    return children_list

  def generate_goal_node(self, node, desired_state):
    traj, traj_distance = construct_dubins_traj(node.state, desired_state)
    desired_node = self.create_node(desired_state, node)

    if(not self.collision_found(node, desired_node)):
      return self.create_node(desired_state, node)
    return None

  def create_node(self, state, parent_node):
    h_cost = self.estimate_cost_to_goal(state)
    g_cost = parent_node.g_cost + self.calculate_edge_distance(state,parent_node)

    return Node(state, parent_node, g_cost, h_cost)

  def create_initial_node(self, state):
    h_cost = self.estimate_cost_to_goal(state)
    g_cost = 0

    return Node(state, None, g_cost, h_cost)

  def calculate_edge_distance(self, state, parent_node):
    traj, traj_distance = construct_dubins_traj(parent_node.state, state)
    if(collision_found(traj, self.objects, self.walls)):
      return A_Star_Planner.LARGE_NUMBER
    else:
      return traj_distance

  def estimate_cost_to_goal(self, state):
    return math.sqrt( (self.desired_state[1] - state[1])**2 + (self.desired_state[2] - state[2])**2 )

  def build_traj(self, goal_node):
    node_list = []
    node_to_add = goal_node
    node_tracker = []

    while node_to_add != None:
      node_list.insert(0, node_to_add)
      node_tracker.insert(0, node_to_add.state)
      node_to_add = node_to_add.parent_node

    print("nodes built up", node_tracker)

    traj = []
    total_dist = 0 
    for i in range(1,len(node_list)):
      node_A = node_list[i-1]
      node_B = node_list[i]
      traj_point_0 = node_A.state
      traj_point_1 = node_B.state
      # TODO: Ask Prof. Clark, what is this doing?
      edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1)
      total_dist += edge_traj_distance
      traj = traj + edge_traj
    print()
    print("TOTAL DISTANCE:", total_dist)
    print()
    return traj, total_dist

  def collision_found(self, node_1, node_2):
    """ Return true if there is a collision with the traj between 2 nodes and the workspace
        Arguments:
          node_1 (Node): A node with the first state of the traj - Time, X, Y, Theta (s, m, m, rad).
          node_2 (Node): A node with the second state of the traj - Time, X, Y, Theta (s, m, m, rad).
          objects (list of lists): A list of object states - X, Y, radius (m, m, m).
          walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
        Returns:
          collision_found (boolean): True if there is a collision.
    """
    traj, traj_distance = construct_dubins_traj(node_1.state, node_2.state)
    return collision_found(traj, self.objects, self.walls)

# # Original Code from Prof. Clark
total_distance = 0
cnt = 0
if __name__ == '__main__':
  for i in range(0, 10):
    maxR = 10
    tp0 = [0, 0, -8, 0]
    tp1 = [300, random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0]
    planner = A_Star_Planner()
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    num_objects = 25
    objects = []
    for j in range(0, num_objects):
      obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
      while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
        obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
      objects.append(obj)
    traj, dist  = planner.construct_traj(tp0, tp1, objects, walls)
    total_distance+= dist
    if(dist >0):
      cnt+=1 
    print("TOTAL DISTANCE OF PLOTS:", total_distance)
    print("TOTAL COUNT", cnt)
    if len(traj) > 0:
      plot_traj(traj, traj, objects, walls)


# Code to generate a setup with known obstacles and goal point instead of random

# if __name__ == '__main__':
#   for i in range(0, 5):
#     maxR = 10
#     tp0 = [0, 0, 0, 0]
#     tp1 = [25, 7, 7, 0]
#     planner = A_Star_Planner()
#     walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
#     num_objects = 2
#     objects = [[5, 7, 1], [9, 7, 1]]

#     # for j in range(0, num_objects):
#     #   obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
#     #   while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
#     #     obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
#     #   objects.append(obj)

#     traj = planner.construct_traj(tp0, tp1, objects, walls)
#     if len(traj) > 0:
#       plot_traj(traj, traj, objects, walls)