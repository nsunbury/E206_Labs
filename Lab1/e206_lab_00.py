import gym
import gym_fetch
import time
import math
import random
from traj_planner_utils import *
from traj_tracker import *

    
def main():
  # Create a motion planning problem and solve it
  current_state, desired_state, objects, walls = create_motion_planning_problem()

  #For Path Tracking
  # desired_traj = [ 
  # [0,0,0,0],[0,2,0,0], 
  # [0,0,0,0],[0,2,2,0], 
  # [0,0,0,0],[0,0,2,np.pi/2], 
  # [0,0,0,0],[0,-2,2,0], 
  # [0,0,0,0],[0,-2,0,0], 
  # [0,0,0,0],[0,0,-2,-np.pi/2], 
  # [0,0,0,0],[0,2,-2,0]
  # ]

  #For Traj Tracking
  desired_traj = construct_dubins_traj(current_state, desired_state)
  
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

  # For Path Tracking
  # reached_current_point = False 
  N = 0 
  while (not traj_tracker.is_traj_tracked() and not controller.is_point_tracked()):
      current_state = [current_time_stamp, observation[0], observation[1], observation[2]]

      # For Path Tracking
      # is_last_point, desired_state = traj_tracker.get_path_to_track(current_state, reached_current_point)
      # reached_current_point, action = controller.point_tracking_control(desired_state, current_state, is_last_point)

      # Traj Tracking
      is_last_point, desired_state = traj_tracker.get_traj_point_to_track(current_state)
      action = controller.point_tracking_control(desired_state, current_state, is_last_point)
      observation, reward, done, dummy = env.step(action)
      env.render('human')
      actual_traj.append(current_state)
      current_time_stamp += TIME_STEP_SIZE
      RMSE_X.append((desired_state[1]-current_state[1])**2)
      RMSE_Y.append((desired_state[2]-current_state[2])**2)
      RMSE_THETA.append((desired_state[3]-current_state[3])**2)
      N += 1 

  RMSE_X_calc = math.sqrt(sum(RMSE_X)/N)
  RMSE_Y_calc = math.sqrt(sum(RMSE_Y)/N)
  RMSE_THETA_calc = math.sqrt(sum(RMSE_THETA)/N)
  
  fig, axs = plt.subplots(3, sharex = True)
  fig.suptitle('RMSE of X, Y, Theta')
  axs[0].plot(range(N),RMSE_X)
  axs[0].set_ylabel("X")
  axs[1].plot(range(N),RMSE_Y)
  axs[1].set_ylabel("Y")
  axs[2].plot(range(N), RMSE_THETA)
  axs[2].set_ylabel("Theta")

  time.sleep(2)
  plot_traj(desired_traj, actual_traj, objects, walls)
  
  env.close()
  
def create_motion_planning_problem():
  current_state = [0, 0, 0, 0]
  desired_state = [1, 0.5, 0, 0]
  # desired_state = [100, 2, 2, np.pi/3]
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  objects = [[4, 0, 1.0], [-2, -3, 1.5]]
  
  return current_state, desired_state, objects, walls

if __name__ == '__main__':
    main()