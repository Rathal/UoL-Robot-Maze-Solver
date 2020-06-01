import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import pi, ceil

class Proximity:
    def __init__(self):
        rospy.init_node('Proximity')

        self.driver_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        self.rotator_sub = rospy.Subscriber('/odom', Odometry, callback=self.odomCB)
        
        self.state_sub = rospy.Subscriber('/state', String, self.stateCB)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)

        # Stores internal twist information
        self.t = Twist()
        self.state = "Waiting" #Used to get Red state from vision

        #Keep its current Yaw angle
        self.yaw = 0

        self.proximity = 0 #Legacy wall avoidance system

        # Internal State System:
        # 0 - loop start, turn left
        # n > 0 - turned left, checking. If not safe turning right
        # -1 - no obstacle in the way. Going forward
        self.stateID = 0
        # Ensures no actions are taken whilst turning besides turning
        self.turning = False
        # Used to set goal angle. -1 means no goal
        self.goal = -1
        # Counts the number of left turns it makes. If it makes 3 turns and is on it's 4th, turn right instead to avoid a loop.
        self.leftTurns = 0

    

    def degree(self, x):
        #Converts Radians to Degrees
        degree = x*180/pi
        if degree > 360: degree -= 360
        return degree
    
    def cycleRadian(self, radian):
        #Ensures radians are never negative. If negative, cycle to positive
        if radian > 2*pi:
            radian -= 2*pi
        return radian

    def getClosestCardinal(self, angle):
        # cardinal = ((angle+(pi/4))//(pi/4))
        # cardinal -= 1
        # cardinal = cardinal * pi/2
        # if cardinal > 2*pi:
        #     cardinal -= 2*pi
        # Legacy Code^^ gets closest cardinal to current yaw angle. Issues with 0/2pi.
        #Above Issues Still Persist, but now don't break the robot from spinning.
        
        # Manually setting each angle range to its respective radian
        if pi/4 < angle <  3*pi/4: cardinal = pi/2
        elif 3*pi/4 < angle < 5*pi/4: cardinal = pi
        elif 5*pi/4 < angle < 7*pi/4: cardinal = 3*pi/2
        else: cardinal = 2*pi
        print 'Cardinal: ', cardinal
        
        return cardinal
        
    def stayTrue(self):
        # Stay True to to closest cardinal direction.
        # If it starts drifting 1 degree (0.0174533 rads) away, it corrects itself

        # Determines cardinal heading to stay facing
        heading = self.getClosestCardinal(self.yaw)
        if heading == 0.0 or heading == 2*pi:
            print 'Good'
        #If Yaw drifts too far from heading, given 1 degree clearance, adjust
        elif self.yaw < heading-0.0174533:
            print 'Adjusting Left'
            self.t.angular.z = 0.1
        elif self.yaw > heading+0.0174533:
            print 'Adjusting Left'
            self.t.angular.z = -0.1
        #drive forward nonetheless
        self.t.linear.x = 0.1
        self.driver_pub.publish(self.t)

    def odomCB(self, odom):
        #Updates the robot's self.yaw variable based on calculated yaw from odometry
        ori = odom.pose.pose.orientation
        ori_list = [ori.x, ori.y, ori.z, ori.w]
        
        #Calculate roll, pitch, yaw, from orientation
        (r, p, y) = euler_from_quaternion(ori_list)            
        yaw = y+pi
        self.yaw = yaw

    def stateCB(self, state):
        #Sets state based on subscription to State topic.
        if state.data != self.state: #Doesn't update if state == published state
            print "updating state"
            self.state = state.data
            print state
    
    def isAheadBlocked(self, range):

        # If red in front of  robot, blocked
        if self.state == "Red": 
            return True
        # If detecting something closer than 1m away, blocked
        elif range < 1:
            return True
        #Else, free to move
        else: return False

    def Go(self, distance):
        #Go forward 1m
        print 'Distance: ', distance

        #If there is room to go
        if distance > 1:
            print 'Good Distance'
            #If Goal hasn't been set before
            if self.goal == -1:
                print 'Setting Goal'
                self.goal = distance - 1
                #Setting Goal to distance - 1m
            #If more than 1m away from Goal, Go
            if distance > self.goal:
                print 'Distance: ', distance, ' Goal: ', self.goal
                self.stayTrue()
            #Else, Stop moving, and reset states.
            else:
                print 'Done'
                self.t.linear.x = 0
                self.t.angular.z = 0
                self.goal = -1
                self.stateID = 0
            print 'Goal: ', self.goal, ' Distance: ', distance
        elif distance > 0.5:
            #Ensure a close fit to the wall as to not be too far away and miss exits
            self.stayTrue()
        #Stop and reset states
        else:
            print 'Done'
            self.t.linear.x = 0
            self.t.angular.z = 0
            self.goal = -1
            self.stateID = 0

    def turnByDegree(self, degree):
        #Declare self as turning
        self.turning = True

        #Define turning clearance as 2 rads
        rad = 2*0.0174533

        #Get Cardinal direction to head towards
        cardinal = self.degree(self.getClosestCardinal(self.yaw))
        
        #If No Goal Set, Set goal
        if self.goal == -1: #Closest Direction + Change -> Radians
            print 'Goal Set'
            self.goal = (cardinal + degree) * pi / 180
            if self.goal > 2*pi: self.goal -= 2*pi
        ##Turn towards goal
        #If missing target, correct yourself
        if self.goal - rad > self.yaw:
            print 'Turning Left'
            self.t.angular.z = 0.35 #0.25
        elif self.goal + rad < self.yaw:
            print 'Turning Right'
            self.t.angular.z = -0.35 #-0.25
        #If within clearance range, stop and reset states.
        else:
            print 'Close Enough'
            self.t.angular.z = 0
            self.stateID += 1
            self.goal = -1
            self.turning = False
        print 'Goal: ', self.goal, ' Yaw: ', self.yaw
        self.driver_pub.publish(self.t)


    def laser_cb(self, laser_msg):
        front = laser_msg.ranges[320]

        # MAIN LOOP
        # Loop is:
        #   TurnLeft()
        #   If NotBlocked:
        #       Go()
        #   Else:
        #       TurnRight()

        if 0 <= self.stateID < 1:
            #Turn +90 degrees (left, Anti-Clockwise)
            self.turnByDegree(90) 
            if self.turning == False:
                #Keep track of how many left turns have been made before a right. This is to stop a loop
                self.leftTurns += 1
            print 'Left Turns: ', self.leftTurns
        #If initial left turn has been completed
        else:
            if self.stateID >= 1:
                print 'Checking Ahead'
                #Check to see if the tile 1m ahead is fit for travel
                blocked = self.isAheadBlocked(front)
                #If robot has made 3 left turns in a row, take a right. 4 lefts is a loop
                if blocked or self.turning or self.leftTurns > 3: 
                    print 'Ahead is Blocked: '
                    #Right turn, so reset left turn counter
                    self.leftTurns = 0
                    #Turn -90 degrees (right, clockwise)
                    self.turnByDegree(-90)
                #If the path ahead is clear, and robot not currently turning
                else:
                    self.stateID = -1
                    print 'Exit Found'
            #If an exit has been found
            elif self.stateID == -1:
                #Go straight ahead 1m
                self.Go(front)

        self.driver_pub.publish(self.t)

proximity = Proximity()
rospy.spin()