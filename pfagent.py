#!/usr/bin/env python
# encoding: utf-8
import sys, math, time
from mybzrc import MyBZRC, Command
from fields import plot

from Bag import Bag

class mask:
    mytanks     = 0
    othertanks  = 1 << 0
    flags       = 1 << 1
    bases       = 1 << 2
    obstacles   = 1 << 3
    shots       = 1 << 4
    attractive  = 1 << 5
    repulsive   = 1 << 6
    tangential  = 1 << 7
    all         = sys.maxint


config = None

agent = None

class BZRException(Exception): pass

#Variables for frobbing the PD controller
PD_Kp = 3
PD_Kd = 2

#Base class of all objects in our world that can produce a potential field,
#like a flag, enemy tank, bullet, etc.
class FieldGenerator:
    #Generates a field for the given x,y position
    def generate_field(self, x, y, m=mask.all):
        pass

        
    def calc_dist(self, x, y, goal_x, goal_y):
        return math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        
    def calc_theta(self, x, y, goal_x, goal_y):
        return math.atan2(goal_y - y, goal_x - x)
        
    def generate_attractive(self, x, y, goal_x, goal_y, alpha, radius, spread):
        dist = self.calc_dist(x, y, goal_x, goal_y)
        theta = self.calc_theta(x, y, goal_x, goal_y)
        
        if dist < radius:
            return 0,0
            
        elif radius <= dist and dist <= spread + radius:
            x = alpha * (dist - radius) * math.cos(theta)
            y = alpha * (dist - radius) * math.sin(theta)
            return x,y
            
        elif dist > spread + radius:
            x = alpha * spread * math.cos(theta)
            y = alpha * spread * math.sin(theta)
            return x,y
        
    def generate_repulsive(self, x, y, goal_x, goal_y, beta, radius, spread):
        dist = self.calc_dist(x, y, goal_x, goal_y)
        theta = self.calc_theta(x, y, goal_x, goal_y)
        
        if dist < radius:
            x = -(beta*(math.cos(theta))*float(10000))
            y = -(beta*(math.sin(theta))*float(10000))
            return x,y
            
        elif radius <= dist and dist <= spread + radius:
            x = -(beta*(spread + radius - dist)*math.cos(theta))
            y = -(beta*(spread + radius - dist)*math.sin(theta))
            return x,y
            
        elif dist > radius + spread:
            return 0,0
        
    def generate_tangential(self, x, y, goal_x, goal_y, gamma, radius, spread):
        dist = self.calc_dist(x, y, goal_x, goal_y)
        #Modify the theta to produce a perpendicular field
        theta = self.calc_theta(x, y, goal_x, goal_y) + (math.pi / 2)
        
        if dist < radius:
            x = -(gamma*(math.cos(theta))*float(100))
            y = -(gamma*(math.sin(theta))*float(100))
            return x,y
            
        elif radius <= dist and dist <= spread + radius:
            x = -(gamma*(spread + radius - dist)*math.cos(theta))
            y = -(gamma*(spread + radius - dist)*math.sin(theta))
            return x,y
            
        elif dist > radius + spread:
            return 0,0


class EnemyTank(FieldGenerator):
    def __init__(self, tank, radius):
        self.status = tank.status
        self.flag = tank.flag
        self.pos_x = tank.x
        self.pos_y = tank.y
        self.radius = radius
        #These variables can be used for frobbing the field
        self.spread = 80
        self.alpha = 0 #Attractive field strength
        self.beta = .5 #Repulsive field strength
        self.gamma = .5 #Tangential field strength
        
    def generate_field(self, x, y, m=mask.all):
        repulsive_x, repulsive_y = self.generate_repulsive(x,y,self.pos_x,self.pos_y,config.EnemyTank.beta,self.radius, config.EnemyTank.spread)
        tangential_x, tangential_y = self.generate_tangential(x,y,self.pos_x,self.pos_y,config.EnemyTank.gamma,self.radius, config.EnemyTank.spread)
        
        retX = 0
        retY = 0
        if m & mask.repulsive:
            retX += repulsive_x
            retY += repulsive_y
        if m & mask.tangential:
            retX += tangential_x
            retY += tangential_y
        return retX, retY

        return (repulsive_x + tangential_x), (repulsive_y + tangential_y) 
        
        
class FriendlyTank(FieldGenerator):
    def __init__(self, tank, radius):
        self.status = tank.status
        self.flag = tank.flag
        self.pos_x = tank.x
        self.pos_y = tank.y
        self.radius = radius
        #These variables can be used for frobbing the field
        self.spread = 50
        self.alpha = 0 #Attractive field strength
        self.beta = .5 #Repulsive field strength
        self.gamma = 1 #Tangential field strength
        
    def generate_field(self, x, y, m=mask.all):
        repulsive_x, repulsive_y = self.generate_repulsive(x,y,self.pos_x,self.pos_y,config.FriendlyTank.beta,self.radius, config.FriendlyTank.spread)
        tangential_x, tangential_y = self.generate_tangential(x,y,self.pos_x,self.pos_y,config.FriendlyTank.gamma,self.radius, config.FriendlyTank.spread)
        
        retX = 0
        retY = 0
        if m & mask.repulsive:
            retX += repulsive_x
            retY += repulsive_y
        if m & mask.tangential:
            retX += tangential_x
            retY += tangential_y
        return retX, retY
        return (repulsive_x + tangential_x), (repulsive_y + tangential_y)
        
        
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
        
    def generate_field(self, x, y, m=mask.all):
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
        self.spread = 10
        self.alpha = 1 #Attractive field strength
        
    def generate_field(self, x, y, m=mask.all):
        #Calculate the center of the base and its width
        width = self.corner1_x - self.corner4_x
        height = self.corner1_y - self.corner2_y
        center_x = self.corner1_x - (width / 2)
        center_y = self.corner1_y - (height / 2)
        #
        radius = (min(width,height) / 2)
        
        if m & mask.attractive:
            return self.generate_attractive(x, y ,center_x, center_y, config.FriendlyBase.alpha, radius, config.FriendlyBase.spread)
        
        return 0, 0
        
class EnemyFlag(FieldGenerator):
    def __init__(self, flag, radius):
        self.pos_x = flag.x
        self.pos_y = flag.y
        self.poss_team = flag.poss_color
        self.radius = radius
        #These variables can be used for frobbing the field
        self.spread = 50
        self.alpha = 1 #Attractive field strength
        
    def generate_field(self, x, y, m=mask.all):
        if m & mask.attractive:
            return self.generate_attractive(x, y, self.pos_x, self.pos_y, config.EnemyFlag.alpha, self.radius, config.EnemyFlag.spread)
        return 0, 0
        
        
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
        
    def generate_field(self, x, y, m=mask.all):
        return 0,0
        
        
class Shot(FieldGenerator):
    def __init__(self, shot, radius):
        self.pos_x = shot.x
        self.pos_y = shot.y
        self.radius = radius
        #These variables can be used for frobbing the field
        self.spread = 100
        self.alpha = 0 #Attractive field strength
        self.beta = 100 #Repulsive field strength
        self.gamma = 1 #Tangential field strength
        
    def generate_field(self, x, y, m=mask.all):
        repulsive_x, repulsive_y = self.generate_repulsive(x,y,self.pos_x,self.pos_y,config.Shot.beta,self.radius, config.Shot.spread)
        tangential_x, tangential_y = self.generate_tangential(x,y,self.pos_x,self.pos_y,config.Shot.gamma,self.radius, config.Shot.spread)

        retX = 0
        retY = 0
        if m & mask.repulsive:
            retX += repulsive_x
            retY += repulsive_y
        if m & mask.tangential:
            retX += tangential_x
            retY += tangential_y
        return retX, retY

        return (repulsive_x + tangential_x), (repulsive_y + tangential_y)
        
        
class Obstacle(FieldGenerator):
    def __init__(self, obstacle):
        self.corners = obstacle
        #These variables can be used for frobbing the field
        self.radius = 100
        self.spread = 100
        self.beta = .5 #Repulsive field strength
        self.gamma = .3 #Tangential field strength
        
    def generate_field(self, x, y, m=mask.all):
        return_x = 0
        return_y = 0
        
        '''#For each corner of the obstacle, create a tangential field
        #to direct the agent around the obstacle
        for corner in self.corners:
            repulsive_x, repulsive_y = self.generate_repulsive(x,y,corner[0],corner[1],self.beta,self.radius, self.spread)
            tangential_x, tangential_y = self.generate_tangential(x,y,corner[0],corner[1],self.gamma,self.radius, self.spread)
            return_x += repulsive_x + tangential_x
            return_y += repulsive_y + tangential_y'''
            
        #Construct a circle around the obstacle
        max_x = float("-inf")
        max_y = float("-inf")
        min_x = float("inf")
        min_y = float("inf")
        for corner in self.corners:
            if corner[0] > max_x:
                max_x = corner[0]
            if corner[1] > max_y:
                max_y = corner[1]
            if corner[0] < min_x:
                min_x = corner[0]
            if corner[1] < min_y:
                min_y = corner[1]
                
            width = max_x - min_x
            height = max_y - min_y
            radius = max(width, height) / 2
            center_x = min_x + (width / 2)
            center_y = min_y + (height / 2)
        
            repulsive_x, repulsive_y = self.generate_repulsive(x,y,center_x,center_y,config.Obstacle.beta,radius, config.Obstacle.spread)
            tangential_x, tangential_y = self.generate_tangential(x,y,center_x,center_y,config.Obstacle.gamma,radius, config.Obstacle.spread)
            #return_x += repulsive_x + tangential_x
            #return_y += repulsive_y + tangential_y
            if m & mask.repulsive:
                return_x += repulsive_x
                return_y += repulsive_y
            if m & mask.tangential:
                return_x += tangential_x
                return_y += tangential_y
        
        return return_x, return_y


class PFAgent:
    def __init__(self, bzrc, index):
        #Variables for interaction with the bzrflag server
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.tank_index = index
        global config
        config = Bag.FromJSON(open('config.py').read())
        
        print "Game constants: {0}".format(self.constants)

        mytanks, othertanks, flags, bases, obstacles, shots = self.bzrc.get_lots_o_stuff()
        self.flags = flags
        self.obstacles = obstacles
        self.shots = shots
        self.bases = bases
        self.mytanks = mytanks
        self.othertanks = othertanks
        
        #The only internal state that we are allowed to keep is the "previous" error for the PD controller
        self.angvel_pd_error = 0
        self.vel_pd_error = 0
        
    #A moment of time has passed.  Calculate the potential field for where we are at, and perform an action
    #based on the field
    def tick(self, time_diff):
        #Find out information about myself
        myself = self.bzrc.get_mytanks()[self.tank_index]
        
        if myself.status == "dead":
            return
        
        #Calculate the potential field at my location
        field_x, field_y = self.calculate_pf(myself.x, myself.y)
        #Perform an action based on the potential field
        self.perform_action(field_x, field_y, time_diff)
        
        
    #Applies a PD controller to a goal, and returns the action and new error.
    def pd_controller(self, goal, cur_value, prev_error, dt, kp, kd):
        #a = kp * (goal - cur_value) + kd * ((goal - cur_value) - prev_error)/dt
        cur_error = self.normalize_angle(goal - cur_value)
        derivative = (cur_error - prev_error) / dt
        result = (kp * cur_error) + (kd * derivative)
        return (result, cur_error)
    
    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle
        
    #Combines all of the behaviors/fields to produce a potential field for the agent's current position
    #If the agent has the flag, the goal is a friendly base, otherwise it is an enemy flag.  The 
    #appropriate behaviors are implemented by adding field generators for the desired objects of the world.
    #For example, an "Avoid enemy" behavior is implemented by adding in an EnemyTank field generator for
    #each enemy tank
    def generate_field_function(self, scale):
        #field_generators = self.create_field_generators(mytanks=self.mytanks, othertanks=self.othertanks, flags=self.flags, bases=self.bases, obstacles=self.obstacles, shots=self.shots)
        field_generators = self.create_field_generators(mytanks=self.mytanks, flags=self.flags, obstacles=self.obstacles)
        def function(x, y):
            '''User-defined field function.'''
            field_x = 0
            field_y = 0
            for field_generator in field_generators:
                delta_x, delta_y = field_generator.generate_field(x,y)
                field_x += delta_x
                field_y += delta_y

            return field_x, field_y
        return function

    def plot(self, m):
        ''' '''
        field_generators = self.create_field_generators(m)
        def create():
            def func(x, y):
                field_x = 0
                field_y = 0
                for field_generator in field_generators:
                    delta_x, delta_y = field_generator.generate_field(x,y,m)
                    field_x += delta_x
                    field_y += delta_y

                return field_x, field_y
            return func
        plot(self, create)

    def calculate_pf(self, x, y):
        field_x = 0
        field_y = 0

        mytanks, othertanks, flags, bases, obstacles, shots = self.bzrc.get_lots_o_stuff()
        self.flags = flags
        self.obstacles = obstacles
        self.shots = shots
        self.bases = bases
        self.mytanks = mytanks
        self.othertanks = othertanks
        #field_generators = self.create_field_generators(mytanks=mytanks, othertanks=othertanks, flags=flags, bases=bases, obstacles=obstacles, shots=shots)
        #field_generators = self.create_field_generators(mytanks=self.mytanks, flags=self.flags, obstacles=self.obstacles)
        field_generators = self.create_field_generators(mask.all)
        global config
        config = Bag.FromJSON(open('config.py').read())
        #self.plot(mask.all)
        #plot(self)

        for field_generator in field_generators:
            delta_x, delta_y = field_generator.generate_field(x,y)
            field_x += delta_x
            field_y += delta_y
            
        return field_x, field_y
            
        
    #Creates a list of field generators that will produce the potential fields
    def create_field_generators(self, m):
        #Get all the information about the world available
        sub_fields = []
        if m & (mask.mytanks | mask.flags):
            myself = self.mytanks[self.tank_index]
            
            #Seek goal behavior differs if we are holding the flag
            if myself.flag != "-":
                #I am holding a flag.  Set the goal to return to my base by creating a FriendlyBase field generator
                for base in self.bases:
                    if base.color == self.constants['team']:
                        sub_fields.append(FriendlyBase(base))
                        break
            else:
                #I don't have a flag.  Set the goal to be an enemy flag by creating a EnemyFlag field generator for
                #each enemy flag
                for flag in self.flags:
                    #if flag.color != self.constants['team']:
                    if flag.color == "green":
                        sub_fields.append(EnemyFlag(flag, float(self.constants['flagradius'])))

        #Add in fields for enemy tanks
        if m & mask.othertanks:
            for tank in self.othertanks:
                sub_fields.append(EnemyTank(tank, float(self.constants['tankradius'])))

        #Add in fields for friendly tanks
        if m & mask.mytanks:
            for tank in self.mytanks:
                if tank.index != self.tank_index:
                    sub_fields.append(FriendlyTank(tank, float(self.constants['tankradius'])))

        #Avoid obstacles
        if m & mask.obstacles:
            for obstacle in self.obstacles:
                sub_fields.append(Obstacle(obstacle))

        #Avoid shots
        if m & mask.shots:
            for shot in self.shots:
                sub_fields.append(Shot(shot, float(self.constants['shotradius'])))

        return sub_fields   
        
        
    #Performs the action based on the potential field calculated (delta_x, delta_y)
    #A PD controller is applied to the desired angle to produce an angular velocity
    def perform_action(self, delta_x, delta_y, time_diff):
        myself = self.bzrc.get_mytanks()[self.tank_index]
        
        #Calculate the velocity and angular velocity to do
        velocity = math.sqrt(delta_x**2 + delta_y**2)
        theta = math.atan2(delta_y, delta_x)
        
        #Run the angle through a pd controller to get angular velocity
        angvel, self.angvel_pd_error = self.pd_controller(theta, myself.angle, self.angvel_pd_error, time_diff, PD_Kp, PD_Kd)
        
        #print "Command: velocity: {0} angvel: {1}".format(velocity, angvel)
        command = [Command(self.tank_index, velocity, angvel, False)]
        self.bzrc.do_commands(command)

if __name__ == '__main__':
    # Process CLI arguments.
    interactive = False
    print sys.argv
    try:
        if len(sys.argv) == 3:
            execname, host, port = sys.argv
        else:
            execname, host, port, interactive = sys.argv
    except ValueError:
        pass
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)
    print 'blah', execname, host, port, interactive

    # Connect.
    #bzrc = MyBZRC(host, int(port), debug=True)
    bzrc = MyBZRC(host, int(port))

    #Create an agent
    global agent
    agent = PFAgent(bzrc, 0)

    if not interactive:
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
