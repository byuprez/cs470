#!/usr/bin/env python
# encoding: utf-8
import sys
import math

from bzrc import BZRC, Command

#Variables for frobbing the PD controller
PD_Kp = 0
PD_Kd = 0

#Base class of all objects in our world that can produce a potential field,
#like a flag, enemy tank, bullet, etc.
class FieldGenerator:
	#Generates a field for the given x,y position
	def generate_field(self, x, y):
		pass

class EnemyTank(FieldGenerator):
	def __init__(self, tank):
		pass
		
class FriendlyTank(FieldGenerator):
	def __init__(self, tank):
		pass
		
class EnemyBase(FieldGenerator):
	def __init__(self, base):
		pass
		
class FriendlyBase(FieldGenerator):
	def __init__(self, base):
		pass
		
class EnemyFlag(FieldGenerator):
	def __init__(self, flag):
		pass
		
class FriendlyFlag(FieldGenerator):
	def __init__(self, flag):
		pass
		
class Shot(FieldGenerator):
	def __init__(self, shot):
		pass
		
class Obstacle(FieldGenerator):
	def __init__(self, obstacle):
		pass

class PFAgent:
	def __init__(self, bzrc, index):
		#Variables for interaction with the bzrflag server
		self.bzrc = bzrc
		self.constants = self.bzrc.get_constants()
		self.commands = []
		self.tank_index = index
		
		#The only internal state that we are allowed to keep is the "previous" error for the PD controller
		self.pd_error = 0
		
	def tick(self, time_diff):
		pass
		
	#Applies a PD controller to a goal, and returns the action and new error.
	def pdController(self, goal, cur_value, prev_error, dt, kp, kd):
		#a = kp * (goal - cur_value) + kd * ((goal - cur_value) - prev_error)/dt
		cur_error = goal - cur_value
		derivative = (cur_error - prev_error) / dt
		result = (kp * cur_error) + (kd * derivative)
		return (result, cur_error)
		
	#Combines all of the behaviors/fields to produce a potential field for the agent's current position
	#If the agent has the flag, the goal is a friendly base, otherwise it is an enemy flag
	def calculate_pf(self):
		pass
		
	#Performs the action based on the potential field calculated (delta_x, delta_y)
	#A PD controller is applied to the desired angle to produce an angular velocity
	def perform_action(self, delta_x, delta_y):
		pass
		

if __name__ == '__main__':
	# Process CLI arguments.
	try:
		execname, host, port = sys.argv
	except ValueError:
		execname = sys.argv[0]
		print >>sys.stderr, '%s: incorrect number of arguments' % execname
		print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
		sys.exit(-1)

	# Connect.
	#bzrc = BZRC(host, int(port), debug=True)
	bzrc = BZRC(host, int(port))

	#Create an agent
	agent = PFAgent(bzrc, 1)

	#Run the agent(s)
	prev_time = time.time()
	try:
		while True:
			cur_time = time.time()
			time_diff = cur_time - prev_time
			agent.tick(time_diff)
			prev_time = cur_time
	except KeyboardInterrupt:
		print "Exiting due to keyboard interrupt."
	