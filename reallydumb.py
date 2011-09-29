#!/usr/bin/python

import sys
import time
import random
import math
from bzrc import BZRC, Command

def enum(**enums):
	return type('Enum', (), enums)

AgentState = enum(MOVING=1, TURNING=2, STOPPED=3)

TURN_GOAL = 1.047 #radians

class ReallyDumbAgent:
	
	def __init__(self, bzrc, index):
		#Information needed to interact with the bzrflag world
		self.bzrc = bzrc
		self.constants = bzrc.get_constants()
		self.tank_index = index
		
		#We need to keep track of a state so we know what to do
		self.state = AgentState.STOPPED
		
		#Variables used for timing
		self.forward_time = 0
		self.forward_time_limit = random.randint(3,8)
		self.fire_time = 0
		
		#Constants for applying a PD controller to our angular velocity
		self.angvel_pd_kp = 5
		self.angvel_pd_kd = 1
		#The error at t - 1 for our angular velocity (needed by the PD controller for the derivative)
		self.angvel_pd_error = 0
		
		#The current goal for our angle, relative to the starting angle of our turn
		self.cur_ang_goal = 0
		
	def normalize_angle(self, angle):
		"""Make any angle be between +/- pi."""
		angle -= 2 * math.pi * int (angle / (2 * math.pi))
		if angle <= -math.pi:
			angle += 2 * math.pi
		elif angle > math.pi:
			angle -= 2 * math.pi
		return angle
	
	#Applies a PD controller to a goal, and returns the action and new error.
	def pdController(self, goal, cur_value, prev_error, dt, kp, kd):
		#a = kp * (goal - cur_value) + kd * ((goal - cur_value) - prev_error)/dt
		cur_error = goal - cur_value
		print "Goal: {0} Cur Val: {1}".format(goal, cur_value)
		derivative = (cur_error - prev_error) / dt
		result = (kp * cur_error) + (kd * derivative)
		print "result: {0}".format(result)
		return (result, cur_error)
	
	#Begins turning
	def begin_turn(self):
		#Get information about myself
		tank_info = self.bzrc.get_mytanks()
		myself = tank_info[self.tank_index]
		
		#Set the goal as my angle + turn goal
		self.cur_ang_goal = self.normalize_angle(myself.angle + TURN_GOAL)
		print "cur angle goal: {0}".format(self.cur_ang_goal)
		#Reset the error of the pd controller
		self.angvel_pd_error = 0
		
		print "Beginning turn"
		self.bzrc.angvel(self.tank_index, 1)
		self.state = AgentState.TURNING
		
	#Begins moving the agent forward	
	def begin_forward_movement(self):
		print "angvel: 0"
		self.bzrc.angvel(self.tank_index, 0)
		print "speed: 1"
		self.bzrc.speed(self.tank_index, 1)
		
		#Reset the time limit and amount of time we have spent moving forward
		self.forward_time = 0
		self.forward_time_limit = random.randint(3,8)
		self.state = AgentState.MOVING
		
	#Runs our turn through a PD controller to correct for errors
	def check_turn(self, time_diff):
		#Get information about myself
		tank_info = self.bzrc.get_mytanks()
		myself = tank_info[self.tank_index]
		
		#Run the turn through the PD controller
		new_angvel, self.angvel_pd_error = self.pdController(self.cur_ang_goal, myself.angle, self.angvel_pd_error, time_diff, self.angvel_pd_kp, self.angvel_pd_kd)
		
		print "new angvel: {0}".format(new_angvel)
		
		if abs(self.angvel_pd_error) <= 0.01:
			#We are close enough to our desired angle.	Stop turning and begin moving forward
			self.begin_forward_movement()
		else:
			#Set the angular velocity to the new value returned by the PD controller
			print "angvel: {0}".format(new_angvel)
			self.bzrc.angvel(self.tank_index, new_angvel)
	
	
	def tick(self, time_diff):
		if self.state == AgentState.STOPPED:
			#We haven't started moving, begin movement
			self.begin_forward_movement()
		elif self.state == AgentState.MOVING:
			if self.forward_time + time_diff >= self.forward_time_limit:
				#Stop moving
				print "Speed: 0"
				self.bzrc.speed(self.tank_index, 0)
				
				#Begin turning
				self.begin_turn()
			else:
				#Keep moving
				self.forward_time += time_diff
		elif self.state == AgentState.TURNING:
			self.check_turn(time_diff)
		
		#Fire after every two seconds
		if self.fire_time + time_diff >= 2:
			print "Fire"
			self.bzrc.shoot(self.tank_index)
			self.fire_time = 0
		else:
			self.fire_time += time_diff
			

def main():
	# Process CLI arguments.
	try:
		execname, host, port = sys.argv
	except ValueError:
		execname = sys.argv[0]
		print >>sys.stderr, '%s: incorrect number of arguments' % execname
		print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
		sys.exit(-1)
	
	#Connect to the server
	bzrc = BZRC(host, int(port))
	
	#Create an agent and begin the game loop
	agent = ReallyDumbAgent(bzrc, 8)
	
	prev_time = time.time()
	try:
		while True:
			cur_time = time.time()
			time_diff = cur_time - prev_time
			agent.tick(time_diff)
			prev_time = cur_time
	except KeyboardInterrupt:
		print "Exiting due to keyboard interrupt."
	
if __name__ == '__main__':
	main()