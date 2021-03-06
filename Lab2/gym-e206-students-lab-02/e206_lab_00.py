import gym
import gym_fetch
import time
import math
import random
from traj_planner_utils import *
from traj_tracker import *
from traj_planner_A_star import *
    
def main():
  # Create a motion planning problem and solve it
  current_state, desired_state, objects, walls = create_motion_planning_problem()
  # desired_traj = construct_dubins_traj(current_state, desired_state)
  
# current_state, desired_state, objects, walls = create_motion_planning_problem()
  planner = A_Star_Planner() 
  desired_traj = planner.construct_traj(current_state, desired_state, objects, walls)

  # Construct an environment
  env = gym.make("fetch-v0") # <-- this we need to create
  env.set_parameters(TIME_STEP_SIZE, objects)
  env.render('human')
  env.reset()

  # Create the trajectory and tracking controller
  controller = PointTracker()
  traj_tracker = TrajectoryTracker(desired_traj)
      
  # Create the feedback loop
  time.sleep(1)
  current_time_stamp = 0
  observation = [0,0,0,0,0]
  actual_traj = []
  RMSE_X = []
  RMSE_Y = []
  RMSE_THETA = []
  N = 0 
  # is_last_point= False
  while not traj_tracker.is_traj_tracked():
    current_state = [current_time_stamp, observation[0], observation[1], observation[2]]
    desired_state = traj_tracker.get_traj_point_to_track(current_state)
    print("Cur:",current_state,"Des:",desired_state)
    action = controller.point_tracking_control(desired_state, current_state)
    observation, reward, done, dummy = env.step(action)
    env.render('human')
    actual_traj.append(current_state)
    current_time_stamp += TIME_STEP_SIZE
    RMSE_X.append((desired_state[1]-current_state[1])**2)
    RMSE_Y.append((desired_state[2]-current_state[2])**2)
    RMSE_THETA.append((desired_state[3]-current_state[3])**2)
    N+=1 

  RMSE_X_calc = math.sqrt(sum(RMSE_X)/N)
  RMSE_Y_calc = math.sqrt(sum(RMSE_Y)/N)
  RMSE_THETA_calc = math.sqrt(sum(RMSE_THETA)/N)
  print("------RMSE-------")
  print(RMSE_X_calc)
  print(RMSE_Y_calc)
  print(RMSE_THETA_calc)
  time.sleep(2)
  plot_traj(desired_traj, actual_traj, objects, walls)
  
  env.close()

def create_motion_planning_problem():
  current_state = [0, 0, 0, 0]
  desired_state = [20, 7, 7, 0]
  maxR = 10
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  objects = [[3, 3, 1], [1, 1, 0.5]]
  
  return current_state, desired_state, objects, walls

if __name__ == '__main__':
    main()
    
    
