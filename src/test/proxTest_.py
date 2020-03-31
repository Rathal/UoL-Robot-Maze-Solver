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
        from math import pi
        print (laser_msg.angle_min *180/pi) +30
        print (laser_msg.angle_max *180/pi) +30
        print '0: ', laser_msg.Ranges[0]
        print '30: ', laser_msg.Ranges[len(laser_msg.Ranges[])/2]
        print '60: ', laser_msg.Ranges[len(laser_msg.Ranges[])]

proximity = Proximity()
rospy.spin()