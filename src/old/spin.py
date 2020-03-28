import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16

class Spinner:
    def __init__(self):
        rospy.init_node('Spinner')
        self.rotator_sub = rospy.Subscriber('/odom', Odometry, callback=self.cb)
        self.spinner_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.state_sub = rospy.Subscriber('/state', Int16, self.stateCB)
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1)
        
        self.state = 0
        self.t = Twist()
        self.origin = 0
        self.rotations = 1
    
    def stateCB(self, state):
        self.state = state

    def cb(self, e):
        import math

        if self.state == 0:

            ori = e.pose.pose.orientation
            orientation_list = [ori.x, ori.y, ori.z, ori.w]
            (r, p, y) = euler_from_quaternion(orientation_list)
            if self.origin == 0:
                self.origin = round(y,2)

            #print y

            #print self.origin
            if self.rotations > 0:
                if (abs(y-self.origin)) > 0.1:
                    print 'Has Moved'
                    self.rotations -= 1
                self.t.angular.z = -1
            elif round(y, 2) == self.origin:
                print 'stopped'
                self.t.angular.z = 0
                self.state_pub.publish(1)

            
            # print "Theta: ", round(theta,2)
            # print "Origin: ", round(self.origin, 2)


            self.spinner_pub.publish(self.t)
        else:
            self.origin = 0
            self.rotations = 1
        

spinner = Spinner()
rospy.spin()