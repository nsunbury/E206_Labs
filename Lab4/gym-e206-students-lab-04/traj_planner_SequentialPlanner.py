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
from traj_planner_ExpPlanner import *
import sys 

class Planning_Problem():
  """ Class that holds a single motion planning problem for one robot.
  """

  def __init__(self, initial_state, desired_state, objects, walls, time_budget):
    """ Constructor for planning problem.
        - Arguments:
          - initial_state (list of floats): The initial state of the robot (t, x, y, theta).
          - desired_state (list of floats): The desired state of the robot (t, x, y, theta)
          - objects (list of list of floats): The objects to avoid.
          - walls (list of list of floats): The walls to not drive through.
    """
    self.initial_state = initial_state
    self.desired_state = desired_state
    self.objects = objects
    self.walls = walls
    self.planner = Expansive_Planner(time_budget)
  
  def get_traj(self, trajs_planned): 
    self.add_trajs_as_obstacles(trajs_planned)
    traj, traj_dist = self.planner.construct_optimized_traj(self.initial_state, self.desired_state, self.objects, self.walls)
    return traj, traj_dist
    
  def add_trajs_as_obstacles(self, traj_set):
    """ Function to add trajs as obstacles to the pp.
        - Arguments
          - traj_set (list of list of floats): The trajectories to be treated as obstacles.
    """
    for traj in traj_set:
      self.objects.append(['traj',traj])

class Sequential_Planner():
  
  def __init__(self):
    pass

  def construct_traj_set(self, planning_problem_list):
    """ Function that creates a set of trajectories for several planning problems (1 per robot).
        - Parameters:
          - planning_problem_list (list of Planning_Problems): One pp to solve per robot.
        - Returns:
          - traj_set (list of lists of floats): One traj per robot.
    """
    trajectories = []
    cnt = 1
    total_path_len = 0
    # Add code here to create a traj set #
    for planning_problem in planning_problem_list: 
      print(cnt)
      cnt+=1 
      traj,traj_dist = planning_problem.get_traj(trajectories)
      total_path_len += traj_dist
      if(len(traj)==0):
        print("TRAJS NOT FOUND, BREAK")
        return [], 0, False
      trajectories.append(traj)
      

    return trajectories, total_path_len, True 
      
def random_pose(maxR):
  """ Function to generate random pose (x, y, theta) within bounds +/- maxR.
      Parameters:
        - maxR (float): the half width of the square environment in meters.
      Returns:
        - random_pose (list of floats): The x, y, theta in (m, m, rad).
  """
  return [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), random.uniform(-math.pi, math.pi)]
  
def get_new_random_pose(pose_list, maxR, radius):
  """ Function to generate random pose (x, y, theta) within bounds +/- maxR.
      Parameters:
        - pose_list (list of floats):
        - maxR (float): the half width of the square environment in meters.
        - radius (float): The objects' radius in meters.
      Returns:
        - random_pose (list of floats): The x, y, theta, radius in (m, m, rad, m).
  """
  new_pose_found = False
  while not new_pose_found:
    new_pose = random_pose(maxR)
    new_pose_found = True
    for pl in pose_list:
      effective_radius = radius + pl[3]
      if (abs(pl[0]-new_pose[1]) < effective_radius and abs(pl[1]-new_pose[2]) < effective_radius) or (abs(pl[0]-new_pose[1]) < effective_radius and abs(pl[1]-new_pose[2]) < effective_radius):
        new_state_found = False
  
  return new_pose + [radius]

def dist(state1, state2): 
  return math.sqrt(((state2[1]-state1[1])**2) + ((state2[2]-state1[2])**2))

def priority_calc(initial_state, objects): 
  cnt = 0 
  for obj in objects: 
    print(obj)
    if(obj[0]=="object" and dist(initial_state, [None, obj[1][0], obj[1][1], None])): 
      cnt+=1 
  return cnt

def tuple_zero(item): 
  return item[0]

def main(argv):
  seed = 25
  random.seed(seed)
  SEQ_TIME_BUDEGET = 30

  num_robots = 4
  num_objects = 15
  maxR = 10
  obj_vel = 1.0
  desired_end_time = 1000

  best_traj_seq= []
  best_traj_dist_seq = 999999

  #Set to STATIC, RANDOM, PRIORITY
  ordering = argv[0]
  print(ordering)
  
  robot_initial_pose_list = []
  robot_initial_state_list = []
  for _ in range(num_robots):
    ns = get_new_random_pose(robot_initial_pose_list, maxR, ROBOT_RADIUS)
    robot_initial_pose_list += [ns]
    robot_initial_state_list += [[0, ns[0], ns[1], ns[2]]]
  
  robot_desired_pose_list = []
  robot_desired_state_list = []
  for _ in range(num_robots):
    ns = get_new_random_pose(robot_desired_pose_list, maxR, ROBOT_RADIUS)
    robot_desired_pose_list += [ns]
    robot_desired_state_list += [[desired_end_time, ns[0], ns[1], ns[2]]]

  object_list = []
  for _ in range(num_objects):
    object_radius = random.uniform(0.3, 1.0)
    obj_pose = get_new_random_pose(robot_initial_pose_list + robot_desired_pose_list, maxR, object_radius)
    obj_yaw = obj_pose[2]
    object_list += [ ['obstacle',[obj_pose[0], obj_pose[1], 0.5, obj_vel, obj_yaw]] ]
    
  robot_initial_state_list_duplicate = copy.deepcopy(robot_initial_state_list)
  robot_desired_state_list_duplicate = copy.deepcopy(robot_desired_state_list)
  object_list_duplicate = copy.deepcopy(object_list)
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  
  success = 0 
  fails = 0 
  total_path_length = 0 
  
  start_time= time.perf_counter()
  seq_cnt = 0
  
  pp_priority  = []

  if(ordering == "PRIORITY"):
    for i in range(num_robots):
      # dist_states = dist(robot_initial_state_list[i], robot_desired_state_list[i])
      # pp_priority.append((dist_states,i))
      obj_priority = priority_calc(robot_initial_state_list[i], object_list)
      pp_priority.append((obj_priority,i))

    pp_priority.sort(key=tuple_zero, reverse= True)
  

  
  while(time.perf_counter()-start_time < SEQ_TIME_BUDEGET):
    planning_problem_list = []
    robot_initial_state_list = copy.deepcopy(robot_initial_state_list_duplicate)
    robot_desired_state_list = copy.deepcopy(robot_desired_state_list_duplicate)
    object_list = copy.deepcopy(object_list_duplicate)

    elapsed_time = (time.perf_counter()-start_time)
    time_left = SEQ_TIME_BUDEGET - elapsed_time
    print(time_left)
    if(ordering == "STATIC"):
      for i in range(num_robots):
        alloted_time  = time_left / (2**(num_robots-i))
        pp = Planning_Problem(robot_initial_state_list[i], robot_desired_state_list[i], object_list, walls, alloted_time)
        planning_problem_list.append(pp)
  
    elif(ordering == "RANDOM"): 
      rangevals = np.arange(num_robots)
      np.random.shuffle(rangevals)
      for i,ind in enumerate(rangevals):
        alloted_time  = time_left / (2**(num_robots-i))
        pp = Planning_Problem(robot_initial_state_list[ind], robot_desired_state_list[ind], object_list, walls, alloted_time)
        planning_problem_list.append(pp)

    elif(ordering == "PRIORITY"): 
      for i,ind in enumerate(pp_priority):
        alloted_time  = time_left / (2**(num_robots-i))
        pp = Planning_Problem(robot_initial_state_list[ind[1]], robot_desired_state_list[ind[1]], object_list, walls, alloted_time)
        planning_problem_list.append(pp)

      
    else: 
      print("ERROR")
      return

    planner = Sequential_Planner()
    traj_list, traj_dist , planned = planner.construct_traj_set(planning_problem_list)
    if(traj_dist < best_traj_dist_seq and len(traj_list)!= 0):
      # print("\n--------------------\n \n \n \n --------------\n")
      # print("traj_list",traj_list[0][0])
      best_traj_seq = copy.deepcopy(traj_list) 
      best_traj_dist_seq = traj_dist
      # print("best_traj",best_traj[0][0])
    
    total_path_length += traj_dist 
    if(planned): 
      seq_cnt+= 1 
      success +=1 
      # plot_traj_list(traj_list, object_list, walls)
    else: 
      fails += 1
  # if(len(best_traj_seq)!=0):
  #   plot_traj_list(best_traj_seq, object_list, walls)
  
  f = open(f"robots{num_robots}_time{SEQ_TIME_BUDEGET}_{ordering}_{seed}.txt", "a")
  f.write("\n----------------------------")
  f.write(f"\nTIME BUDGET:{SEQ_TIME_BUDEGET}")
  f.write(f"\nNO OF ROBOTS: {num_robots}")
  f.write(f"\nNO OF OBJECTS: {num_objects}, OBJ_VELOCITY: {obj_vel}")
  f.write(f"\nTOTAL PATH LENGTH: {total_path_length}")
  if(seq_cnt!=0):
    f.write(f"\nAVERAGE PATH LENGTH: {(total_path_length/seq_cnt)}")
  f.write(f"\nBEST PATH LENGTH: {best_traj_dist_seq}")
  f.write(f"\nSUCCESSES: {success}, FAILURES:{fails}\n")

  print("TIME BUDGET:", SEQ_TIME_BUDEGET)
  print("NO OF ROBOTS", num_robots)
  print(f"NO OF OBJECTS: {num_objects}, OBJ_VELOCITY: {obj_vel}")
  if(seq_cnt!=0):
    print("AVERAGE PATH LENGTH", (total_path_length/seq_cnt))
  print("SUCCESSES:", success, " FAILURES: ", fails)
  if(success!=0):
    return best_traj_dist_seq
  return -1

if __name__ == '__main__':
  N = 50
  results = {}
  orderings= ["PRIORITY", "STATIC", "RANDOM"]
  f = open(f"RESULTS_robots{4}_time{30}.txt", "a")
  for order in orderings:
    path_sum = 0
    for i in range(N):
      path_sum+= main([order])
    results[order] = (path_sum/N)
    f.write(f"\n{order} with {N} runs: {results[order]}m average")

  
  print("Number of Runs", N)
  print(results)
  
  # main(["STATIC"])