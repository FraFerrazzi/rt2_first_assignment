from __future__ import print_function
import time
from sr.robot import *
import os
import __builtin__

max_grab_radius = sim_robot.GRAB_RADIUS

R = Robot()
""" instance of the class Robot"""

def print(message):
  """
  Redifining print to be prettier and to keep the console always clear,
  displaying only one message at a time
  """
  os.system('clear')
  __builtin__.print("\033[92m")
  __builtin__.print(" /=================\\")
  __builtin__.print(" | ROBOT SIMULATOR |")
  __builtin__.print(" \\=================/")
  __builtin__.print("\033[96m")
  __builtin__.print(" Robot: " + message)

def stop():
  """
  Function for halting the robot's movement
  """
  R.motors[0].m0.power = 0
  R.motors[0].m1.power = 0

def drive(speed):
  """
  Function for setting a linear velocity (without stopping)
  Args: speed (int): the speed of the wheels
  """
  R.motors[0].m0.power = speed
  R.motors[0].m1.power = speed

def turn(speed, direction):
  """
  Function for setting an angular velocity (without stopping)
  Args: speed (int): the speed of the wheels
        direction (string): 'LEFT' or 'RIGHT'
  """
  if (direction == 'RIGHT'):
    # turn right
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed/4
  elif (direction == 'LEFT'):
    # turn left
    R.motors[0].m0.power = speed/4
    R.motors[0].m1.power = speed

def turn180(speed):
  """
  Turn 180 degrees clockwise
  R.heading values:
    0deg = 0
    90deg = -1.5
    180deg = -3 = +3
    270deg = 1.5
  Args: speed (int): the speed of the wheels
  """
  # these calculations are necessary to work in the angular [-3; +3] range that
  # the R.heading attribute returns
  max_error = 0.2
  start_heading = R.heading
  curr_heading = R.heading
  discrepancy = abs(start_heading) # |(start_heading + 3) - 3|
  goal_heading = -3 + discrepancy

  # correcting heading if value negative
  if (start_heading < 0):
    goal_heading *= -1

  while(abs(curr_heading - goal_heading) > max_error):
    curr_heading = R.heading
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(0.01)

  stop()

def find_token_cone(max_deg):
  """
  Function to find the closest token in restricted cone of angle max_deg
  Args: max_deg (float): cone size to search in (directed in front of robot)
  Returns:
  dist (float): distance of the closest token (-1 if no token is detected)
  rot_y (float): angle between the robot and the token (-1 if no token is detected)
  is_silver: the closest token is silver
  """
  dist_init = 10
  dist = dist_init
  is_silver = 0

  for token in R.see():
    if token.dist < dist and (-max_deg < token.rot_y < max_deg):
        dist=token.dist
        rot_y=token.rot_y
        if token.info.marker_type == MARKER_TOKEN_SILVER:
          is_silver = 1

  if dist == dist_init:
    return -1, -1
  else:
    return dist, rot_y, is_silver

def turn_from_wall(rot_speed):
  """
  Function to turn towards the correct path when encountering a wall.
  Scan left and right and turn towards the direction with the farthest away
  average gold marker distance
  """

  # cone starts from -90 and +90 degrees and "fans out" by cone_deg_back towards
  # the "back" of the robot, and by cone_deg_front by towards the front of the
  # robot
  cone_deg_back = 0
  cone_deg_front = 90

  max_dist = 2
  avg_dist_left = 0
  avg_dist_right = 0
  token_count_left = 0
  token_count_right = 0

  for token in R.see():
    # LEFT side tokens
    if (token.dist < max_dist and token.info.marker_type is MARKER_TOKEN_GOLD and
       (-90-cone_deg_back < token.rot_y < -90+cone_deg_front)):
      avg_dist_left += token.dist
      token_count_left = token_count_left + 1

    # RIGHT side tokens
    if (token.dist < max_dist and token.info.marker_type is MARKER_TOKEN_GOLD and
       (+90-cone_deg_front < token.rot_y < +90+cone_deg_back)):
      avg_dist_right += token.dist
      token_count_right = token_count_right + 1

  # calculating average distance
  if (token_count_left > 0):
    avg_dist_left /= token_count_left
  else:
    # if no tokens detected, set distance to max because path is free
    avg_dist_left = max_dist
  if (token_count_right > 0):
    avg_dist_right /= token_count_right
  else:
    # if no tokens detected, set distance to max because path is free
    avg_dist_right = max_dist

  # if tokens in front are far away, turn at slower speed (smooth behaviour)
  dist_front, rot_y, is_silver = find_token_cone(30)
  if (dist_front > 1.4):
    rot_speed = rot_speed/2;

  # turning towards best direction
  if (avg_dist_left > avg_dist_right):
    # turn left
    print("Turning left with speed " + str(rot_speed));
    turn(rot_speed, 'LEFT')
  else:
    # turn right
    print("Turning right with speed " + str(rot_speed));
    turn(rot_speed, 'RIGHT')

def grab_and_turn(turn_speed):
  """
    Grabs a token, turns 180 degrees, places it down, turns back
  """

  print("Moving silver token")

  while (R.grab() == False):
    print("Grabbing token")
    # in case the robot doesn't grab the token
    drive(-20)
    time.sleep(1)
    return -1

  turn180(turn_speed)
  R.release()
  drive(-turn_speed) # avoids hitting the token when we turn back
  time.sleep(0.3)
  turn180(turn_speed)

def turn_to_silver(max_search_deg, max_rot_error_deg, rot_speed):
  dist, rot_y, is_silver = find_token_cone(max_search_deg)

  if (rot_y > 0):
    # if token on right, turn right
    turn(rot_speed, 'RIGHT')
    print("Aligning to silver")
  else:
    # if token on left, turn left
    turn(rot_speed, 'LEFT')
    print("Aligning to silver")

def main():
  """
  CONFIG
  """
  max_search_deg = 30 # forward scanner cone angle
  fwd_speed = 100
  rot_speed = 100
  max_rot_error_deg = 3 # acceptable error for silver token alignment
  max_obstacle_dist = 1.5 # max distance before obstacle avoidance kicks in

  # let the simulator load... (eliminates startup lag)
  print("Booting...")
  time.sleep(1)

  # main simulator loop
  while(1):
    dist, rot_y, is_silver = find_token_cone(max_search_deg)

    if (is_silver):
      # if found silver token ahead
      if (dist < max_grab_radius):
        # if token close, grab it
        grab_and_turn(rot_speed)
        stop()
      else:
        # otherwise, turn to face it and drive towards it
        if (abs(rot_y) > max_rot_error_deg):
          turn_to_silver(max_search_deg, max_rot_error_deg, rot_speed/3)
        else:
          drive(fwd_speed)
          print("Driving to silver")

    else:
      # if found GOLD token ahead
      if (dist > max_obstacle_dist):
        # drive until too close
        drive(fwd_speed)
        print("Driving...")

      else:
        # if too close, scan left and right and drive to farthest golden token
        turn_from_wall(rot_speed)

    time.sleep(0.05)

main()
