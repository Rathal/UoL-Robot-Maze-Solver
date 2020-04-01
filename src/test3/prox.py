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

        self.t = Twist()
        self.state = "Waiting"

        self.yaw = 0
        self.proximity = 0

        self.rotateDegree = 0

        self.foundWall = False
        self.stateID = 0
        self.turning = False
        self.goal = -1
        self.leftTurns = 0

    

    def degree(self, x):
        degree = x*180/pi
        if degree > 360: degree -= 360
        return degree
    
    def cycleRadian(self, radian):
        if radian > 2*pi:
            radian -= 2*pi
        return radian

    def getClosestCardinal(self, angle):
        # cardinal = ((angle+(pi/4))//(pi/4))
        # cardinal -= 1
        # cardinal = cardinal * pi/2
        # if cardinal > 2*pi:
        #     cardinal -= 2*pi            
        if pi/4 < angle <  3*pi/4: cardinal = pi/2
        elif 3*pi/4 < angle < 5*pi/4: cardinal = pi
        elif 5*pi/4 < angle < 7*pi/4: cardinal = 3*pi/2
        #else: cardinal = 0
        # elif 0 < angle < pi/4: cardinal = 0
        # elif 7*pi/4 < angle < 2*pi: cardinal = 2*pi
        else: cardinal = 2*pi
        print 'Cardinal: ', cardinal
        
        return cardinal
    
    #HALF WORKING
    def faceAngle(self, angle):
                
        print 'Yaw: ', self.yaw
        angle = abs(angle)

        if ceil(self.degree(angle)) == 360: angle = 0
        if ceil(self.degree(angle)) == -360: angle = 0
        print 'Angle: ', angle

        self.goal = angle
        #if round(self.goal, 1) == round(yaw, 1):
        if self.goal == 0:
            self.t.angular.z = 0
            self.state_pub.publish(String(data="Done"))
            self.stateID += 0.5
            self.turning = False
        elif self.goal > self.yaw:
            print 'Left'
            self.t.angular.z = 0.25
        elif self.goal < self.yaw:
            print 'Right'
            self.t.angular.z = -0.25
        self.driver_pub.publish(self.t)

    # def faceAngle(self, angle):
        
    #     print 'Yaw: ', self.yaw
    #     if ceil(self.degree(angle)) == 360: angle = 0
    #     print 'Angle: ', angle

    #     if self.goal < 0:
    #         self.goal = angle

    #     #if round(self.goal, 1) == round(yaw, 1):
    #     if self.goal == round(self.yaw,-1):
    #         self.t.angular.z = 0
    #         self.state_pub.publish(String(data="Done"))
    #         self.stateID += 0.5
    #         self.turning = False
    #         self.goal = -1
    #     elif self.goal > self.yaw:
    #         print 'Left'
    #         self.t.angular.z = 0.25
    #     elif self.goal < self.yaw:
    #         print 'Right'
    #         self.t.angular.z = -0.25
    #     self.driver_pub.publish(self.t)

# HALF WORKING
    def turnDegree(self, degree):
        if degree == 90: degree = 270
        elif degree == 270: degree = 90

        radian = degree*pi /180
        if radian > 0:
            radian += self.yaw
        else: 
            radian = (self.yaw - radian) + 2*pi
        radian = self.cycleRadian(radian)
        print 'Degrees: ', degree, ' Pos: ', self.yaw*180/pi, ' Goal :', radian*180/pi
        self.faceAngle(radian)

    # def turnDegree(self, degree):
    #     #I want to face degree (degrees)
    #     print 'I want to face: ', degree
    #     print 'Current yaw: ', self.degree(self.yaw)
    #     radian = degree * pi / 180
    #     self.faceAngle(radian)

    def obstacleDetected(self):
        import random

        #proximity = self.state.lstrip('Obstacle Detected: ')
        cardinal = self.degree(self.getClosestCardinal(self.yaw))
        #newCardinal = cardinal + (iProximity*90) #SHOULD BE -
        if bool(random.getrandbits(1)): newCardinal = cardinal +90
        else: newCardinal = cardinal -90
        #newCardinal = cardinal +90
        if newCardinal < 0: newCardinal += 360
        elif newCardinal > 360: newCardinal -= 360
        return int(newCardinal)
        
    def stayTrue(self):
        heading = self.getClosestCardinal(self.yaw)
        if heading == 0.0 or heading == 2*pi:
            'ERROR INCOMING'
        elif self.yaw < heading-0.0174533:
            print 'Adjusting Left'
            self.t.angular.z = 0.1
            # self.driver_pub.publish(self.t)
        elif self.yaw > heading+0.0174533:
            print 'Adjusting Left'
            self.t.angular.z = -0.1
            # self.driver_pub.publish(self.t)
        self.t.linear.x = 0.1
        self.driver_pub.publish(self.t)
        # self.state_pub.publish(String(data="TEST"))
        #else: self.t.angular.z = 0
        #else: self.t.angular.z = -0.1

    def odomCB(self, odom):

        ori = odom.pose.pose.orientation
        ori_list = [ori.x, ori.y, ori.z, ori.w]
        #Calculate roll, pitch, yaw, from orientation
        (r, p, y) = euler_from_quaternion(ori_list)            
        yaw = y+pi
        self.yaw = yaw

    def stateCB(self, state):
        if state.data != self.state:
            print "updating state"
            self.state = state.data
            print state

    def Chunk(self, list, n):
        for i in xrange(0, len(list), n):
            yield list[i:i+n]
        
    def determineProximity(self, distance, Ranges):
        if any(r < distance for r in Ranges[0]):
            print 'Too Close Right'
            self.proximity = 1
        elif any(r < distance for r in Ranges[1]):
            print 'Too Close Left'
            self.proximity = -1
        else: self.proximity = 0

    def turnLeft(self):
        if self.stateID == 0:
            print 'Setting Degree'
            self.rotateDegree = self.degree(self.getClosestCardinal(self.yaw)) +90
            if self.rotateDegree > 360: self.rotateDegree-= 360
            self.stateID = 0.5
        print 'Turning Left: ', self.rotateDegree        
        self.turnDegree(self.rotateDegree)
    
    def turnRight(self):
        if self.stateID.is_integer():
            self.rotateDegree = self.degree(self.getClosestCardinal(self.yaw)) -90
            print 'Rotate Degree: ', self.rotateDegree
            if self.rotateDegree < 0: self.rotateDegree += 360
            self.stateID += 0.5
        print 'Turning Right: ', self.rotateDegree        
        self.turnDegree(self.rotateDegree)

    def isAheadBlocked(self, range):
        if self.state == "Red":
            return True
        elif range < 1:
            return True
        else: return False

    def Go(self, distance):
        print 'Distance: ', distance
        if distance > 1:
            print 'Good Distance'
            if self.goal == -1:
                print 'Setting Goal'
                self.goal = distance - 1
            if distance > self.goal:
                print 'Distance: ', distance, ' Goal: ', self.goal
                self.stayTrue()
            else:
                print 'Done'
                self.t.linear.x = 0
                self.t.angular.z = 0
                self.goal = -1
                self.stateID = 0
            print 'Goal: ', self.goal, ' Distance: ', distance
        elif distance > 0.5:
            self.stayTrue()
        else:
            print 'Done'
            self.t.linear.x = 0
            self.t.angular.z = 0
            self.goal = -1
            self.stateID = 0

    def turnByDegree(self, degree):
        self.turning = True
        rad = 2*0.0174533
        cardinal = self.degree(self.getClosestCardinal(self.yaw))
        #If No Goal Set, Set goal
        if self.goal == -1: #Closest Direction + Change -> Radians
            print 'Goal Set'
            self.goal = (cardinal + degree) * pi / 180
            if self.goal > 2*pi: self.goal -= 2*pi
        ##Turn towards goal
        #if self.goal != 2*pi or self.goal != 0:
        if self.goal - rad > self.yaw:
            print 'Turning Left'
            self.t.angular.z = 0.35 #0.25
        elif self.goal + rad < self.yaw:
            print 'Turning Right'
            self.t.angular.z = -0.35 #-0.25
        else:
            print 'Close Enough'
            self.t.angular.z = 0
            self.stateID += 1
            self.goal = -1
            self.turning = False
        print 'Goal: ', self.goal, ' Yaw: ', self.yaw
        self.driver_pub.publish(self.t)


    def laser_cb(self, laser_msg):
        distance = 0.5
        Ranges = list(self.Chunk(laser_msg.ranges, 320))
        
        if self.state != "Stop": self.determineProximity(distance, Ranges)


        # if 0 <= self.stateID < 1:
        #     print 'Turning'
        #     self.turnByDegree(90)


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
                blocked = self.isAheadBlocked(Ranges[1][0])
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
                self.Go(Ranges[1][0])
                
        self.driver_pub.publish(self.t)

proximity = Proximity()
rospy.spin()