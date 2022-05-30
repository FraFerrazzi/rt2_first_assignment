from __future__ import print_function

import time
from sr.robot import *

"""
Assignment 1 python script

Main code is defined after the definition of the functions. The code should make the robot:
	- 1) move in an autonomus way around the arena
	- 2) avoid the contact with the golden markers (tokens)
	- 3) detect the silver markers (tokens) while it's moving around the arena
	- 4) go towards, grab, move behind itsel and realise the silver markers once they are in range
	- 5) move around the arena following a counter-clockwise direction
	- 6) keep doing the tasks from 1) to 5) until the program is shut down

The method see() of the class Robot returns an object whose attribute info.marker_type may be MARKER_TOKEN_GOLD or MARKER_TOKEN_SILVER,
depending of the type of marker (golden or silver).

1- retrieve the distance and the angle of the closest golden marker placed in front of the robot. If no golden marker is detected, the robot should go straight.
2- retrieve the distance and the angle of the closest silver marker. If the silver marker is inside a given range, the robot detects it, go towards it, grabs it and places it behind itself.
3- move the robot around the arena following counter-clockwise direction. If a golden marker is too close the robot should avoid it.
4- when the robot is too close to a golden marker, in order not to collide into it, should decide in which dircetion it should go: turn left or turn right.
5- if golden markers placed on the right of the robot are further away than the onces on the left, the robot should turn right.
6- if golden markers placed on the right of the robot are closer than the onces on the left, the robot should turn right.
7- keep checking the situation and make the process autonomus.

	When done, run with:
	$ python run.py solutions/exercise3_solution.py

"""


lin_th_silver = 1.1
""" float: Threshold for the control of the linear distance from silver token"""

ang_th_silver = 1.5
""" float: Threshold for the control of the orientation from silver token"""

lin_th_gold = 0.85
""" float: Threshold for the control of the linear distance from golden token"""

lin_th_gold_collide = 1.1
""" float: Threshold for the control of the linear distance from golden token used in the function called find closest token collide"""

th_side_left_ub = -55.0
""" float: Threshold upper bound for the control of the angular orientation of the left side of the robot in respect to its front"""

th_side_right_lb = +55.0
""" float: Threshold lower bound for the control of the angular orientation of the right side of the robot in respect to its front"""

th_side_left_lb = -130.0
""" float: Threshold lower bound for the control of the angular orientation of the left side of the robot in respect to its front"""

th_side_right_ub = +130.0
""" float: Threshold upper bound for the control of the angular orientation of the right side of the robot in respect to its front"""

th_front_right = +60
""" float: Threshold for the control of angular orientation of the front right side of the robot in respect to its front"""

th_front_left = -60
""" float: Threshold for the control of angular orientation of the front left side of the robot in respect to its front"""

th_front_collide_right = +25
""" float: Threshold for the control of angular orientation of the front left side of the robot in respect to its front"""

th_front_collide_left = -25
""" float: Threshold for the control of angular orientation of the front left side of the robot in respect to its front"""

gold_poss_collide = False
""" boolean: variable for letting the robot know if there is a golden token in his way and could collide into it"""

turn_right = False
""" boolean: variable for letting the robot know if it has to turn right in order to avoid a golden token"""

turn_left = False
""" boolean: variable for letting the robot know if it has to turn left in order to avoid a silver token"""

R = Robot()
""" instance of the class Robot"""

def drive(speed, seconds):
    """
    Function for setting a linear velocity

    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def drive_circle_left(speed, seconds):
    """
    Function for setting a rototraslatin velocity

    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = 1.2*speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def drive_circle_right(speed, seconds):
    """
    Function for setting a linear and angular velocity

    Args: speed (int): the speed of the wheels
      seconds (int): the time interval
    """
    R.motors[0].m0.power = 1.2*speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity

    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def find_silver_token():
    """
    Function to find the closest silver token

    Returns:
    dist (float): distance of the closest silver token (-1 if no silver token is detected)
    rot_y (float): angle between the robot and the silver token (-1 if no silver token is detected)
    """
    dist=100
    for token in R.see():
        if token.dist < dist and th_front_left < token.rot_y < th_front_right and token.info.marker_type is MARKER_TOKEN_SILVER:
            dist=token.dist
	    rot_y=token.rot_y
    if dist==100:
	return -1, -1
    else:
   	return dist, rot_y

def find_closest_golden_token_collide():
    """
    Function to find if in front of the robot there is a token that could possibly lead to a collision

    Returns:
    out (boolean): true if a golden token is detected in a given range (distance and angle orientation). false if no token is detected in the range
    """
    dist=lin_th_gold_collide
    out = False
    for token in R.see():
        	if th_front_collide_left < token.rot_y < th_front_collide_right and token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
            		out = True
            		return out
    else:
    	return out

def find_closest_golden_token_front():
    """
    Function to find the closest golden token in front of the robot (in between of -60 and +60 degrees)

    Returns:
    dist (float): distance of the closest golden token in front of the robot (100 if no golden token is detected)
    rot_y (float): angle between the robot and the closest golden token placed in front of him (-1 if no golden token is detected)
    """
    dist=100
    for token in R.see():
		if  th_front_left < token.rot_y < th_front_right and token.info.marker_type is MARKER_TOKEN_GOLD:
        		if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
            			dist=token.dist
	    			rot_y=token.rot_y
    if dist==100:
	return -1, -1
    else:
   	return dist, rot_y

def find_closest_golden_token_left():
    """
    Function to find the closest golden token on the left of the robot (in between -130 and -55 degrees)

    Returns:
    dist (float): distance of the closest golden token on the left of the robot (-1 if no golden token is detected)
    rot_y (float): angle between the robot and the closest golden token placed on the left of him (-1 if no golden token is detected)
    """
    dist=100
    for token in R.see():
		if  th_side_left_lb < token.rot_y < th_side_left_ub and token.info.marker_type is MARKER_TOKEN_GOLD:
        		if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
            			dist=token.dist
	    			rot_y=token.rot_y
    if dist==100:
	return -1, -1
    else:
   	return dist, rot_y

def find_closest_golden_token_right():
    """
    Function to find the closest golden token on the right of the robot (in between +55 and +130 degrees)

    Returns:
    dist (float): distance of the closest golden token on the right of the robot (-1 if no golden token is detected)
    rot_y (float): angle between the robot and the closest golden token placed on the right of him (-1 if no golden token is detected)
    """
    dist=100
    for token in R.see():
		if  th_side_right_lb < token.rot_y < th_side_right_ub and token.info.marker_type is MARKER_TOKEN_GOLD:
        		if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
            			dist=token.dist
	    			rot_y=token.rot_y
    if dist==100:
	return -1, -1
    else:
   	return dist, rot_y
   	
def avoid_collision():
    '''
    Function which make a decision in order to make the robot turn left or right when the robot gets too close to a golden token.    
    '''
    print("I'm to close to a golden token! Let's not collide into it!!!")

    dist_gold_left, angle_gold_left = find_closest_golden_token_left() 
    dist_gold_right, angle_gold_right = find_closest_golden_token_right() 
    print("Let's make a decision:")
    turn_left = False # just making sure to reset variable
    turn_right = False # just making sure to reset variable
    
    if dist_gold_left >= dist_gold_right: # if golden token on the left is further than the one on the right i should turn right
	  print("i should turn left!")
	  turn_left = True
	  while turn_left:
		turn(-20, 0.1) # turn left until no token are detected in his front
		print("turn left a bit...")
		dist_gold_front, angle_gold_front = find_closest_golden_token_front() 
		if  dist_gold_front > lin_th_gold: # if the robot doesn't detect any token within a given range in its front, drives straight a little on the left and exits the loop.
			print("Should be right oriented, let's to straight a little")
			drive_circle_left(20, 0.5) # go straight and turns left a little in the same time... is supposed to keep the distances from the avoided row of golden tokens
			turn_left = False

    else: # if golden token on the left is closer than the one on the right i should turn right
	  print("i should turn right!")
	  turn_right = True
	  while turn_right:
		turn(20, 0.1) # turn right until no token are detected in his front
		print("turn right a bit...")
		dist_gold_front, angle_gold_front = find_closest_golden_token_front()
		if  dist_gold_front > lin_th_gold: # if the robot doesn't detect any token within a given range in its front, drives straight a little on the right and exits the loop.
			print("Should be right oriented, let's to straight a little")
			drive_circle_right(20, 0.5) # go straight and turns right a little in the same time... is supposed to keep the distances from the avoided row of golden tokens
			turn_right = False

def grab_silver():
    '''
    Function to grab the silver token if the robot is close enough and put it behind itself. If the robot is not close enough, it goes towards the silver token in order to grab it.
    '''
    if R.grab(): # if the robot is close enough to grab the token i turn of 180 degrees, release the token, turn of 180 degrees again and go back to the initial position
	  dist_gold_left, angle_gold_left = find_closest_golden_token_left()
	  dist_gold_right, angle_gold_right = find_closest_golden_token_right()
	  print("Got it!!")
	  print("Let's place it behind")
	  if dist_gold_left > dist_gold_right: # check in which direction the robot should turn. if the golden marker on the left is further than the one on the right the robot turns left in order to avoid the possible collision with markers, else it turns right.
		turn(-40, 1.6)
	  else:
		turn(40, 1.6)
	  drive(40, 0.2)
	  R.release()
	  print("Realise the silver token")
	  print("Let's get back to work")
	  drive(-40, 0.5)
	  if dist_gold_left > dist_gold_right: # according to what was detected at the beginning of the if statement, the robot turns in the same direction
		turn(40, 1.6)
	  else:
		turn(-40, 1.6)

    else: # not close enough to grab the token. the robot go towards the silver token in order to grab it
	  print("I'm not close enough to the silver token! Let's get closer")
	  if -ang_th_silver <= angle_silver <= ang_th_silver: # if the robot is alligned in rispect to t silver token i want to grab, it goes forward
		print("I'm alligned! Let's go forward")
		drive(40, 0.5)
	  elif angle_silver < -ang_th_silver: # if the robot is not well aligned with the silver token, we make it turn left a bit
		print("Should go left a bit")
		turn(-5, 0.2)
	  elif angle_silver > ang_th_silver: # if the robot is not well aligned with the silver token, we make it turn left a bit
		print("Should go left a bit")
		turn(5, 0.2)
   

while 1:

	dist_gold_front, angle_gold_front = find_closest_golden_token_front() # get the position of the robot in respect to the closest golden token placed in front of it at ecah cycle
    	dist_silver, angle_silver = find_silver_token() # get the position of the robot in respect to the closest silver token at ecah cycle
    	gold_poss_collide = find_closest_golden_token_collide() # set the boolean variable in order to see if there could be a possible collision or not

    	if dist_gold_front < lin_th_gold and gold_poss_collide: # if we are too close to a golden token and it could lead to a possible collision we need to avoid it
		avoid_collision()

    	elif dist_silver < lin_th_silver:  # if robot is close to a silver token we try to grab the silver token
		grab_silver()

	else: # if the robot is not close to a silver token nor a gold token, it goes straight
		drive(40, 0.1)
		print("No obstacle detected! Let's go straight")
