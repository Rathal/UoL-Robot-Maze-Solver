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
        #print 'Running'
        if self.state == "Drive":
            distance = 0.5
            Ranges = list(self.Chunk(laser_msg.ranges, 320))
            
            self.determineProximity(distance, Ranges)

            if self.proximity != 0:
                msg = "Obstacle Detected: " + str(self.proximity)
                self.state_pub.publish(String(data=msg))

proximity = Proximity()
rospy.spin()