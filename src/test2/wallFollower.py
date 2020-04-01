import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np

class Proximity:
    def __init__(self):
        rospy.init_node('Proximity')
        self.driver_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        
        self.state_sub = rospy.Subscriber('/state', String, self.stateCB)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)

        self.t = Twist()


    def Chunk(self, list, n):
        for i in xrange(0, len(list), n):
            yield list[i:i+n]

    def stateCB(self, state):
        a = 0

    def laser_cb(self, l):
        Ranges2dp = [round(r, 2) for r in l.ranges]
        # for i in range(10):
        #     print i, ': ', round(ranges[i],3), ' | ', i+213, ': ', round(ranges[i+213],3), ' | ', i+426, ': ', round(ranges[i+426],3)

        Ranges = list(self.Chunk(l.ranges, 60))
        


        # self.driver_pub.publish(self.t)




proximity = Proximity()
rospy.spin()