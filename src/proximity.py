import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from kobuki_msgs.msg import BumperEvent


class Chatter:

    def __init__(self):
        rospy.init_node('chatter')
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        
        #self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumperCallback)
        self.state_sub = rospy.Subscriber('/state', Int16, self.stateCB)
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1)

        self.t = Twist()
        self.state = 0
        self.proximity = 0

    def stateCB(self, state):
        self.state = state.data

    def Chunk(self, list, n):
        for i in xrange(0, len(list), n):
            yield list[i:i+n]

    def laser_cb(self, laser_msg):
        #print 'Running'

        distance = 0.5

        Ranges = list(self.Chunk(laser_msg.ranges, 320))
        if all(r < 1.0 for r in laser_msg.ranges):
            print 'Trapped'
            self.proximity = 2
        elif any(r < distance for r in Ranges[0]):
            print 'Too Close Right'
            self.proximity = 1
        elif any(r < distance for r in Ranges[1]):
            print 'Too Close Left'
            self.proximity = -1
        else: self.proximity = 0

        if self.proximity != 0:
            self.state_pub.publish(1)
        else:
            self.state_pub.publish(0)
        print self.state
        
        if self.state == 1:

            if self.proximity == 1:
                self.t.angular.z = 0.5
                self.t.linear.x = 0.0
            elif self.proximity == -1:
                self.t.angular.z = -0.5
                self.t.linear.x = 0.0
            elif self.proximity == 0:
                self.t.angular.z = 0
                self.t.linear.x = 0.2

            self.publisher.publish(self.t)

    def run(self):
        rospy.spin()

c = Chatter()
c.run()

#States
# 0 = go
# 1 = Proximity
# 2 = Head to wall