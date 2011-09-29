#!/usr/bin/env python
# encoding: utf-8
import sys
import math
import time
from mybzrc import MyBZRC, Command

class BZRException(Exception): pass

#Variables for frobbing the PD controller
PD_Kp = 2
PD_Kd = 2

#Base class of all objects in our world that can produce a potential field,
#like a flag, enemy tank, bullet, etc.
class FieldGenerator:
    #Generates a field for the given x,y position
    def generate_field(self, x, y):
        pass
        
    def generate_attractive(self, x, y):
        pass
        
    def generate_repulsive(self, x, y):
        pass
        
    def generate_tangential(self, x, y):
        pass


class EnemyTank(FieldGenerator):
    def __init__(self, tank, radius):
        self.status = tank.status
        self.flag = tank.flag
        self.pos_x = tank.x
        self.pos_y = tank.y
        self.radius = radius
        #These variables can be used for frobbing the field
        self.spread = 0
        self.alpha = 0 #Attractive field strength
        self.beta = 0 #Repulsive field strength
        self.gamma = 0 #Tangential field strength
        
    def generate_field(self, x, y):
        return 0,0
        
        
class FriendlyTank(FieldGenerator):
    def __init__(self, tank, radius):
        self.status = tank.status
        self.flag = tank.flag
        self.pos_x = tank.x
        self.pos_y = tank.y
        self.radius = radius
        #These variables can be used for frobbing the field
        self.spread = 0
        self.alpha = 0 #Attractive field strength
        self.beta = 0 #Repulsive field strength
        self.gamma = 0 #Tangential field strength
        
    def generate_field(self, x, y):
        return 0,0
        
        
class EnemyBase(FieldGenerator):
    def __init__(self, base):
        self.corner1_x = base.corner1_x
        self.corner1_y = base.corner1_y
        self.corner2_x = base.corner2_x
        self.corner2_y = base.corner2_y
        self.corner3_x = base.corner3_x
        self.corner3_y = base.corner3_y
        self.corner4_x = base.corner4_x
        self.corner4_y = base.corner4_y
        #These variables can be used for frobbing the field
        self.spread = 0
        self.alpha = 0 #Attractive field strength
        self.beta = 0 #Repulsive field strength
        self.gamma = 0 #Tangential field strength
        
    def generate_field(self, x, y):
        return 0,0
        
        
class FriendlyBase(FieldGenerator):
    def __init__(self, base):
        self.corner1_x = base.corner1_x
        self.corner1_y = base.corner1_y
        self.corner2_x = base.corner2_x
        self.corner2_y = base.corner2_y
        self.corner3_x = base.corner3_x
        self.corner3_y = base.corner3_y
        self.corner4_x = base.corner4_x
        self.corner4_y = base.corner4_y
        #These variables can be used for frobbing the field
        self.spread = 100
        self.alpha = 1.5 #Attractive field strength
        self.beta = 0 #Repulsive field strength
        self.gamma = 0 #Tangential field strength
        
    def generate_field(self, x, y):
        return self.generate_attractive_field(x,y)
        
    def generate_attractive_field(self, x, y):
        width = self.corner1_x - self.corner4_x
        height = self.corner1_y - self.corner2_y
        center_x = self.corner1_x - (width / 2)
        center_y = self.corner1_y - (height / 2)
        radius = min(width,height)
        
        dist = math.sqrt((center_x - x)**2 + (center_y - y)**2)
        theta = math.atan2(center_y - y, center_x - x)
        
        if dist < radius:
            return 0,0
            
        elif radius <= dist and dist <= self.spread + radius:
            x = self.alpha * (dist - radius) * math.cos(theta)
            y = self.alpha * (dist - radius) * math.sin(theta)
            return x,y
            
        elif dist > self.spread + radius:
            x = self.alpha * self.spread * math.cos(theta)
            y = self.alpha * self.spread * math.sin(theta)
            return x,y
        
        
class EnemyFlag(FieldGenerator):
    def __init__(self, flag, radius):
        self.pos_x = flag.x
        self.pos_y = flag.y
        self.poss_team = flag.poss_color
        self.radius = radius
        #These variables can be used for frobbing the field
        self.spread = 100
        self.alpha = 1.5 #Attractive field strength
        self.beta = 0 #Repulsive field strength
        self.gamma = 0 #Tangential field strength
        
    def generate_field(self, x, y):
        return self.generate_attractive(x,y)
        
    def generate_attractive(self, x, y):
        dist = math.sqrt((self.pos_x - x)**2 + (self.pos_y - y)**2)
        theta = math.atan2(self.pos_y - y, self.pos_x - x)
        
        if dist < self.radius:
            return 0,0
            
        elif self.radius <= dist and dist <= self.spread + self.radius:
            x = self.alpha * (dist - self.radius) * math.cos(theta)
            y = self.alpha * (dist - self.radius) * math.sin(theta)
            return x,y
            
        elif dist > self.spread + self.radius:
            x = self.alpha * self.spread * math.cos(theta)
            y = self.alpha * self.spread * math.sin(theta)
            return x,y
        
class FriendlyFlag(FieldGenerator):
    def __init__(self, flag, radius):
        self.pos_x = flag.x
        self.pos_y = flag.y
        self.poss_team = flag.poss_color
        self.radius = radius
        ##These variables can be used for frobbing the field
        self.spread = 0
        self.alpha = 0 #Attractive field strength
        self.beta = 0 #Repulsive field strength
        self.gamma = 0 #Tangential field strength
        
    def generate_field(self, x, y):
        return 0,0
        
        
class Shot(FieldGenerator):
    def __init__(self, shot, radius):
        self.pos_x = shot.x
        self.pos_y = shot.y
        self.radius = radius
        #These variables can be used for frobbing the field
        self.spread = 0
        self.alpha = 0 #Attractive field strength
        self.beta = 0 #Repulsive field strength
        self.gamma = 0 #Tangential field strength
        
    def generate_field(self, x, y):
        return 0,0
        
        
class Obstacle(FieldGenerator):
    def __init__(self, obstacle):
        self.corners = obstacle
        #These variables can be used for frobbing the field
        self.spread = 0
        self.alpha = 0 #Attractive field strength
        self.beta = 0 #Repulsive field strength
        self.gamma = 0 #Tangential field strength
        
    def generate_field(self, x, y):
        return 0,0


class PFAgent:
    def __init__(self, bzrc, index):
        #Variables for interaction with the bzrflag server
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.tank_index = index
        
        print "Game constants: {0}".format(self.constants)
        
        #The only internal state that we are allowed to keep is the "previous" error for the PD controller
        self.pd_error = 0
        
        
    #A moment of time has passed.  Calculate the potential field for where we are at, and perform an action
    #based on the field
    def tick(self, time_diff):
        #Find out information about myself
        myself = self.bzrc.get_mytanks()[self.tank_index]
        
        #Calculate the potential field at my location
        field_x, field_y = self.calculate_pf(myself.x, myself.y)
        #Perform an action based on the potential field
        self.perform_action(field_x, field_y, time_diff)
        
        
    #Applies a PD controller to a goal, and returns the action and new error.
    def pd_controller(self, goal, cur_value, prev_error, dt, kp, kd):
        #a = kp * (goal - cur_value) + kd * ((goal - cur_value) - prev_error)/dt
        cur_error = goal - cur_value
        derivative = (cur_error - prev_error) / dt
        result = (kp * cur_error) + (kd * derivative)
        return (result, cur_error)
        
        
    #Combines all of the behaviors/fields to produce a potential field for the agent's current position
    #If the agent has the flag, the goal is a friendly base, otherwise it is an enemy flag.  The 
    #appropriate behaviors are implemented by adding field generators for the desired objects of the world.
    #For example, an "Avoid enemy" behavior is implemented by adding in an EnemyTank field generator for
    #each enemy tank
    def calculate_pf(self, x, y):
        field_x = 0
        field_y = 0
        field_generators = self.create_field_generators()
        
        for field_generator in field_generators:
            delta_x, delta_y = field_generator.generate_field(x,y)
            field_x += delta_x
            field_y += delta_y
            
        return field_x, field_y
            
        
    #Creates a list of field generators that will produce the potential fields
    def create_field_generators(self):
        #Get all the information about the world available
        mytanks, othertanks, flags, bases, obstacles, shots = self.bzrc.get_lots_o_stuff()
        myself = mytanks[self.tank_index]
        sub_fields = []

        #Seek goal behavior differs if we are holding the flag
        if myself.flag != "-":
            #I am holding a flag.  Set the goal to return to my base by creating a FriendlyBase field generator
            for base in bases:
                if base.color == self.constants['team']:
                    sub_fields.append(FriendlyBase(base))
                    break

        else:
            #I don't have a flag.  Set the goal to be an enemy flag by creating a EnemyFlag field generator for
            #each enemy flag
            for flag in flags:
                if flag.color != self.constants['team']:
                    sub_fields.append(EnemyFlag(flag, float(self.constants['flagradius'])))
                    break

        #Add in fields for enemy tanks
        for tank in othertanks:
            sub_fields.append(EnemyTank(tank, float(self.constants['tankradius'])))

        #Add in fields for friendly tanks
        for tank in mytanks:
            if tank.index != self.tank_index:
                sub_fields.append(FriendlyTank(tank, float(self.constants['tankradius'])))

        #Avoid obstacles
        for obstacle in obstacles:
            sub_fields.append(Obstacle(obstacle))

        #Avoid shots
        for shot in shots:
            subFields.append(Shot(shot, float(self.constants['shotradius'])))

        return sub_fields   
        
        
    #Performs the action based on the potential field calculated (delta_x, delta_y)
    #A PD controller is applied to the desired angle to produce an angular velocity
    def perform_action(self, delta_x, delta_y, time_diff):
        myself = self.bzrc.get_mytanks()[self.tank_index]
        
        #Calculate the velocity and angular velocity to do
        velocity = math.sqrt(delta_x**2 + delta_y**2)
        theta = math.atan2(delta_y, delta_x)
        angvel, self.pd_error = self.pd_controller(theta, myself.angle, self.pd_error, time_diff, PD_Kp, PD_Kd)
        
        print "Command: velocity: {0} angvel: {1}".format(velocity, angvel)
        command = [Command(self.tank_index, velocity, angvel, False)]
        self.bzrc.do_commands(command)

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
    #bzrc = MyBZRC(host, int(port), debug=True)
    bzrc = MyBZRC(host, int(port))

    #Create an agent
    agent = PFAgent(bzrc, 0)

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
    