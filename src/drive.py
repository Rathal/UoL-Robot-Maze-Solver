import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class DriveModule:
    def __init__(self):
        rospy.init_node('driver')
        self.rotator_sub = rospy.Subscriber('/odom', Odometry, callback=self.cb)
        self.rotator_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    
    def cb(self, e):
        #print e.twist.twist.angular.z
        angular = Twist()
        move = Twist()
        angular = e.twist.twist.angular
        if abs(angular.z) < 1:
            move.linear.x = 0.2
        
        self.rotator_pub.publish(move)


driver = DriveModule()
rospy.spin()