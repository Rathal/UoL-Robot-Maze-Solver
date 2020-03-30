import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Driver:
    def __init__(self):
        rospy.init_node('Driver')
        self.state_sub = rospy.Subscriber('/state', String, self.stateCB)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)
        
        self.driver_sub = rospy.Subscriber('/odom', Odometry, callback=self.odomCB)
        self.driver_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.state = "Null"
        self.t = Twist()

    def stateCB(self, state):
        print "updating state"
        self.state = state.data
        print state

    def odomCB(self, odom):
        if self.state == "Drive":
            print 'Driving'
            self.Drive(0.2)
        else:
            self.t.linear.x = 0
            self.driver_pub.publish(self.t)
            
    
    def Drive(self, speed):
        self.t.linear.x = speed
        self.driver_pub.publish(self.t)

driver = Driver()
rospy.spin()