# E206 Motion Planning

# Simple planner
# C Clark

import math
import dubins
import random
import matplotlib.pyplot as plt
from traj_planner_utils import *
import numpy as np
import time
import copy

class Node():

  def __init__(self, state, parent_node, edge_distance):
    self.state = state
    self.parent_node = parent_node
    self.edge_distance = edge_distance
    
  def manhattan_distance_to_node(self, node):
    return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])
  
  def manhattan_distance_to_state(self, state):
    return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])
    
  def euclidean_distance_to_state(self, state):
    return math.sqrt( (self.state[1] - state[1])**2 + (self.state[2] - state[2])**2 )

  def print(self):
    """ Debugging method: prints all aspects of a node, handling None types properly
    """
    if(self != None):
      state = self.state
      edge_distance = self.edge_distance
    else:
      state = "None"
      edge_distance = "None"
    
    if(self.parent_node != None):
      parent = self.parent_node.state
    else:
      parent = "None"
    
    print("State:", state, " Parent:", parent, " Edge Distance:", edge_distance)


class Expansive_Planner():
  
  DISTANCE_DELTA = 1.5 #m
  DIST_TO_GOAL_THRESHOLD = 0.5 #m
  GRID_RESOLUTION = 0.5 #m
  EDGE_TIME = 10 #s
  LARGE_NUMBER = 9999999
  
  MAX_NUM_ITERATIONS = 1000
  MIN_RAND_DISTANCE = 1 #m
  MAX_RAND_DISTANCE = 5 #m
  MEAN_EDGE_VELOCITY = 0.75 #m
 
    
  def __init__(self, plan_time_budget=0.5):#, tree_size_limit=200, sample_attempt_limit=10000):
    self.rng = np.random.default_rng() #to generate a random number rfloat = self.rng.random()  
    self.PLAN_TIME_BUDGET = plan_time_budget #s

    
    

  def construct_traj(self, initial_state, desired_state, objects, walls, start_time):
    """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
          traj_cost (float): The path length (m).
    """
    self.tree = []
    self.desired_state = desired_state
    self.objects = objects
    self.walls = walls

    current_node = Node(initial_state, None, 0)
    self.add_to_tree(current_node)
    goal = self.generate_goal_node(current_node, self.desired_state)

    while(goal is None): 
      current_node = self.generate_random_node(self.sample_random_node())
      i = 0
      while(current_node is None): 
        if(time.perf_counter()-start_time > self.PLAN_TIME_BUDGET):
          break
        current_node = self.generate_random_node(self.sample_random_node())
        i += 1

      if(time.perf_counter()-start_time > self.PLAN_TIME_BUDGET):
          break
      
      self.add_to_tree(current_node)
      goal = self.generate_goal_node(current_node, self.desired_state)
      
    if(goal is None):  
      return [], self.LARGE_NUMBER
    
    return self.build_traj(goal)
    
  def construct_optimized_traj(self, initial_state, desired_state, objects, walls):
    """ Construct the best trajectory possible within a limited time budget.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          best_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
          best_traj_cost (float): The path lenght of the shortest traj (m).
    """
    start_time = time.perf_counter()
    best_traj = []
    best_traj_cost = self.LARGE_NUMBER
    total_trials = 0
    successful_trials = 0

    while (time.perf_counter()-start_time < self.PLAN_TIME_BUDGET):
      total_trials +=1 
      traj, traj_dist = self.construct_traj(initial_state, desired_state, objects, walls, start_time)
      if(traj_dist < self.LARGE_NUMBER):
        successful_trials += 1
      if(traj_dist < best_traj_cost): 
        best_traj = traj 
        best_traj_cost = traj_dist 
      
    
    # if(best_traj == []):
    #   print("NO PATHS FOUND in ", time.perf_counter()-start_time, "seconds")
    # else:
    #   print(successful_trials, "PATHS FOUND in ", time.perf_counter()-start_time, "seconds")
    return best_traj, best_traj_cost, total_trials, successful_trials
    
  def add_to_tree(self, node):
    """ Add the node to the tree.
        Arguments:
          node (Node): The node to be added.
    """
    self.tree.append(node)
    return 
    
  def sample_random_node(self):
    """ Randomly select a node from the tree and return it.
        Returns:
          node (Node): A randomly selected node from the tree.
    """
    #Naive Approach     
    return self.tree[int(self.rng.random()*len(self.tree))] # OUT OF BOUNDS ERRORS? Check this
    
  def generate_random_node(self, node_to_expand):
    """ Create a new node by expanding from the parent node using.
        Arguments:
          node_to_expand (Node): The parent node.
        Returns:
          new_node (Node): The newly generated node.
    """    
    rand_angle = angle_diff(self.rng.random() * 2 * np.pi - np.pi)
    rand_dist =  self.MIN_RAND_DISTANCE + self.rng.random() * (self.MAX_RAND_DISTANCE- self.MIN_RAND_DISTANCE)

    parent_node_time, parent_node_x, parent_node_y, parent_node_theta = node_to_expand.state
    random_state = []
    random_state.append(parent_node_time + rand_dist/self.MEAN_EDGE_VELOCITY)
    random_state.append(parent_node_x + rand_dist * np.cos(angle_diff(parent_node_theta + rand_angle)))
    random_state.append(parent_node_y + rand_dist * np.sin(angle_diff(parent_node_theta + rand_angle)))
    random_state.append(angle_diff(parent_node_theta + 2 * rand_angle))

    rand_node = Node(random_state, node_to_expand, rand_dist)
    
    if(not self.collision_found(node_to_expand, rand_node)):
      return rand_node

    return None

  def generate_goal_node(self, node, desired_state):
    """ Create a goal node by connecting from the parent node using.
        Arguments:
          node_to_expand: The parent node.
        Returns:
          goal_node: The newly generated goal node or None if there is not goal connection.
    """
    _, traj_distance = construct_dubins_traj(node.state, desired_state)
    desired_node = Node(desired_state, node, traj_distance)
    
    if(not self.collision_found(node, desired_node)):
      return desired_node

    return None

  def calculate_edge_distance(self, state, parent_node):
    """ Calculate the cost of an dubins path edge from a parent node's state to another state.
        Arguments:
          state: The end state of the edge.
          parent_node: The initial state node of the edge.
        Returns:
          traj_distance: The length of the edge, or is the LARGE_NUMBER if collision exists (m).
    """
    traj, traj_distance = construct_dubins_traj(parent_node.state, state)
    if collision_found(traj, self.objects, self.walls):
      return self.LARGE_NUMBER

    return traj_distance

  def build_traj(self, goal_node):
    """ Build a traj via back tracking from a goal node.
        Arguments:
          goal_node: The node to back track from and create a traj of dubins paths with.
        Returns:
          traj (list of list of floats): The trajectory as a list of time, X, Y, Theta (s, m, m, rad).
          traj_cost (float): The length of the traj (m).
    """
    node_list = []
    node_to_add = goal_node
    while node_to_add != None:
      node_list.insert(0, node_to_add)
      node_to_add = node_to_add.parent_node
  
    traj = []
    traj_cost = 0
    for i in range(1,len(node_list)):
      node_A = node_list[i-1]
      node_B = node_list[i]
      traj_point_0 = node_A.state
      traj_point_1 = node_B.state
      edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1)
      traj = traj + edge_traj
      traj_cost = traj_cost + edge_traj_distance
    return traj, traj_cost

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
    traj, _ = construct_dubins_traj(node_1.state, node_2.state)
    return collision_found(traj, self.objects, self.walls)

if __name__ == '__main__':
  
  plan_budgets = [0.001, 0.01, 0.05, 0.1, 0.25, 0.5, 1, 2.5, 5]

  N = 200

  for trial in range(len(plan_budgets)):
    planner = Expansive_Planner(plan_budgets[trial])
    print("Planning Time Budget: ", plan_budgets[trial])
    

    tot_dist = 0
    total_trys = 0 
    successful_trys = 0
    failures = 0
    time_exceeded_count = 0

    for i in range(0, N):
      # print("Trial #", i)
      
      maxR = 10
      tp0 = [0, -9, -9, 0]
      tp1 = [300, 9, 9, 0]
      walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
      num_objects = 30
      objects = [[7.300744881735806, -3.4625745325439246, 1.0], [3.8395292711090523, -0.8818064621920527, 1.0], [-6.023859778546766, -8.636106387738197, 1.0], [8.60967175714691, -0.7317647868509081, 1.0], [-6.726803706089509, 1.404413448327718, 1.0], [-1.2338800806273484, 8.044835525870948, 1.0], [-3.1936037757137345, -3.45521530678933, 1.0], [2.5029518681525715, 4.583665493193516, 1.0], [-1.5438726461909784, -4.515987863722199, 1.0], [-1.8982329663512783, -3.1563897547165887, 1.0], [-0.4355319804104365, 1.9853852874394384, 1.0], [-1.0312556395825965, 8.138283713552457, 1.0], [3.1353998005407426, -1.7720269827649346, 1.0], [4.735819486673154, 7.302123381357145, 1.0], [-0.9542953474752842, -4.090133293937513, 1.0], [0.44060864130242017, 3.4664969795405547, 1.0], [1.825678503415098, -7.811670794890416, 1.0], [4.383551061073588, 8.343577873987368, 1.0], [-2.0512064497973608, 4.288975610254326, 1.0], [6, 1.5, 1]]
      # objects  = []
      # for j in range(0, num_objects): 
      #   obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 1.0]
      #   while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
      #     obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 1.0]
      #   objects.append(obj)
      traj, traj_cost, total_trials, successful_trials  = planner.construct_optimized_traj(tp0, tp1, objects, walls)
     

      if(traj_cost < Expansive_Planner.LARGE_NUMBER):
        tot_dist += traj_cost
      else:
        failures += 1

      total_trys+= total_trials
      successful_trys += successful_trials
      
      # if len(traj) > 0:
      #   plot_traj(traj, traj, objects, walls)
    
    dist_divisor = N - failures
    if (dist_divisor == 0):
      dist_divisor = 1
    # print("dist divisor ", dist_divisor)

    print("Avg. Final Path Distance: ", end="")
    print(tot_dist, "/", dist_divisor, "=", tot_dist/dist_divisor)
    print("Avg. no of attempts per trial: ", end="")
    print(total_trys, "/", N, "=", total_trys/N)
    print("Path Finding Success Rate: ", end="")
    print(successful_trys, "/", total_trys, "=", successful_trys/total_trys)
    print()
