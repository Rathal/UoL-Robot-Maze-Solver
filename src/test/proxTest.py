import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#from kobuki_msgs.msg import BumperEvent

class Proximity:
    def __init__(self):
        rospy.init_node('Proximity')
        self.driver_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        
        #self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumperCallback)
        self.state_sub = rospy.Subscriber('/state', String, self.stateCB)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)

        self.t = Twist()
        self.state = "Null"
        self.proximity = 0

        self.foundWall = False

    def stateCB(self, state):
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

    def laser_cb(self, laser_msg):
        #import numpy as np
        import math
        distance = 0.4
        Ranges = list(self.Chunk(laser_msg.ranges, 320))
        
        self.determineProximity(distance, Ranges)

        print '[0][0]: ', Ranges[0][0]
        print '[0][320]: ', Ranges[0][319]
        print '[1][0]: ', Ranges[1][0]
        print '[1][320]: ', Ranges[1][319]

        if self.proximity == 1 or self.state == "Red":
            print 'Prox 1'
            self.t.angular.z = 0.1
            self.t.linear.x = 0
        elif self.proximity == -1:
            print 'Prox -1'
            self.t.angular.z = 0.1
            self.t.linear.x = 0
        else:
            
            if any(r < distance for r in Ranges[0]):
                print 'Stay True 1'
                self.t.angular.z = 0.25
                # self.t.linear.x = 0.15
            elif any(r > distance + 0.1 for r in Ranges[1]):
                print 'Stay True -1'
                self.t.angular.z = -0.15
                self.t.linear.x = 0.15
            else:
                print 'Else'
                self.t.angular.z = -0.1
                self.t.linear.x= 0.15
            if self.foundWall == True:
                if Ranges[0][0] > distance+0.3:
                    print 'Stay True 1+'
                    self.t.angular.z = 0.5
                    self.t.linear.x = 0
            # print 'Else'
            # self.t.linear.x = 0.1
        # if Ranges[0][0] > 2 and Ranges[0][319] > 0.5 and self.state != "Red":
        #     print 'MAXED'
        #     self.t.angular.z = -1


        if any(0 < r < 0.3 for r in Ranges[0]) or math.isnan(Ranges[0][319]):
            self.t.angular.z = 2.5
            self.state_pub.publish(String(data="Stop"))
        else:
            self.state_pub.publish(String(data="Drive"))

        self.driver_publisher.publish(self.t)

proximity = Proximity()
rospy.spin()