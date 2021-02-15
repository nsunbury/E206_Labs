import time
import math
import random
import numpy as np
from traj_planner_utils import *

TIME_STEP_SIZE = 0.01 #s
LOOK_AHEAD_TIME = 1.0 #s
MIN_DIST_TO_POINT = 0.1 #m
MIN_ANG_TO_POINT = 0.10 #rad

class TrajectoryTracker():
  """ A class to hold the functionality for tracking trajectories.
      Arguments:
        traj (list of lists): A list of traj points Time, X, Y, Theta (s, m, m, rad).
  """
  current_point_to_track = 0
  traj_tracked = False
  traj = []

  def __init__(self, traj):
    self.current_point_to_track = 0
    self.traj = traj
    self.traj_optimized = traj
    self.traj_tracked = False

  def get_path_to_track(self, current_state, reached_current_state):
    """ Determine which point of the traj should be tracked at the current time.
        Arguments:
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
        Returns:
          desired_state (list of floats: The desired state to track - Time, X, Y, Theta (s, m, m, rad).
    """
    if(len(self.traj_optimized)==1): 
      return True, self.traj_optimized[0]
    else: 
      if(reached_current_state):
        self.traj_optimized = self.traj_optimized[1:]
      return False, self.traj_optimized[0]
      
  def get_traj_point_to_track(self, current_state):
    """ Determine which point of the traj should be tracked at the current time.
        Arguments:
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
        Returns:
          desired_state (list of floats: The desired state to track - Time, X, Y, Theta (s, m, m, rad).
    """
    if(len(self.traj_optimized)==1): 
      return True, self.traj_optimized[0]
    else: 
      if(current_state[0] > self.traj_optimized[0][0]):
        self.traj_optimized = self.traj_optimized[1:]
      return False, self.traj_optimized[0]
    # self.current_point_to_track = 0

    # if(current_state[0] > self.traj[-1][0]):
    #   self.current_point_to_track = len(self.traj)-1
    #   return True, self.traj[self.current_point_to_track]

    # for i,traj_point in enumerate(self.traj): 
    #   if(traj_point[0]> current_state[0]):
    #     self.current_point_to_track = i
    #     print(self.traj[self.current_point_to_track])
    #     is_last_point = (i == len(self.traj))
    #     return is_last_point, self.traj[self.current_point_to_track]

    # print(self.traj[self.current_point_to_track])
    # return True, self.traj[self.current_point_to_track]
  
  def print_traj(self):
    """ Print the trajectory points.
    """
    print("Traj:")
    for i in range(len(self.traj)):
        print(i,self.traj[i])
          
  def is_traj_tracked(self):
    """ Return true if the traj is tracked.
        Returns:
          traj_tracked (boolean): True if traj has been tracked.
    """
    return self.traj_tracked
    
class PointTracker():
  """ A class to determine actions (motor control signals) for driving a robot to a position.
  """
  def __init__(self):
    self.point_tracked = False

  def is_point_tracked(self): 
    return self.point_tracked

  def get_dummy_action(self, x_des, x):
    """ Return a dummy action for now
    """
    action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    return action

  def point_tracking_control(self, desired_state, current_state, is_last_point):
    """ Return the motor control signals as actions to drive a robot to a desired configuration
        Arguments:
          desired_state (list of floats): The desired Time, X, Y, Theta (s, m, m, rad).
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
    """
    delta_t = desired_state[0] - current_state[0]
    delta_x = desired_state[1] - current_state[1]
    delta_y = desired_state[2] - current_state[2]
    theta = current_state[3]

    alpha = angle_diff(-theta + np.arctan2(delta_y, delta_x))
    rho = np.sqrt(delta_x**2 + delta_y**2)
    beta = angle_diff(-theta - alpha + desired_state[3])

    # For Path Tracking
    # reached_current_point = False

    if (rho < MIN_DIST_TO_POINT and beta < MIN_ANG_TO_POINT):
      if(is_last_point):
        self.point_tracked = True 
      
      #For Path Tracking
      # reached_current_point = True

    # print("orig:", alpha, beta)
    if(alpha < (-np.pi/2) or alpha > (np.pi/2)):
      # Updated angles 
      alpha = angle_diff(-theta + np.arctan2(-delta_y, -delta_x))
      rho = -rho
      beta = angle_diff(-theta - alpha - desired_state[3])
      # print("updated:", alpha, beta)


    #Propotional Constants
    k_r = 7
    k_b = -10
    k_a = 10 # k_a = 250

    if(rho < 0.25):
      k_r *= 2
      k_a *= 2
      
    v = k_r * rho
    w = k_a * alpha + k_b * beta

    #Using equations from lecture 2A of E205, we solved for wheel velocities in terms of our PCs
    v1 = v + w
    v2 = v - w 

    # zero all of action
    action = [v1, v2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]

    #For Traj Tracking
    return action

    # For Path Tracking
    # return reached_current_point, action
