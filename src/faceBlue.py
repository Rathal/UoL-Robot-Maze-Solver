import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class BlueRotator:
    def __init__(self):
        rospy.init_node('blue_rotator')
        self.blueError_sub = rospy.Subscriber('/error/blue', Float64, callback=self.cb)
        self.rotator_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    
    def cb(self, e):
        print e.data
        error = e.data
        angle = 0
        t = Twist()
        if error > 10.0:
            angle = -0.1
            print '>'
            t.angular.z = angle
        elif error < -10.0:
            angle = 0.1
            print '<'
            t.angular.z = angle
        else:
            angle = 0
            print '='
            t.angular.z = angle

        self.rotator_pub.publish(t)

blueRotator = BlueRotator()
rospy.spin()