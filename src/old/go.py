import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Driver:
    def __init__(self):
        rospy.init_node('blue_rotator')
        self.rotator_sub = rospy.Subscriber('/odom', Odometry, callback=self.cb)
        self.rotator_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

        self.t = Twist()
        self.moved = False
        self.origin = 0

        self.turning = False
        self.tooClose = False
        self.proximity = 0
        self.origin = 0

    def Chunk(self, list, n):
        for i in xrange(0, len(list), n):
            yield list[i:i+n]

    def laser_cb(self, laser):
        distance = 0.5

        Ranges = list(self.Chunk(laser.ranges, 320))
        if any(r < distance for r in Ranges[0]):
            print 'Too Close Right'
            self.proximity = 1
        elif any(r < distance for r in Ranges[1]):
            print 'Too Close Left'
            self.proximity = -1
        else: self.proximity = 0


    def getYaw(self, e):
        ori = e.pose.pose.orientation
        orientation_list = [ori.x, ori.y, ori.z, ori.w]
        (r, p, y) = euler_from_quaternion(orientation_list)
        return round(y,4)

    def driveForward(self, e, limit, direction):
        x = round(e.pose.pose.position.x,4)
        if self.moved == False:
            self.origin = x
            self.moved = True
        print 'Origin: ', self.origin
        print 'X: ', x
        print self.origin - x

        if abs((self.origin - x)) < limit*0.25:
            print 'Driving'
            self.t.linear.x = 0.1 * direction
        else:
            self.t.linear.x = 0
        self.rotator_pub.publish(self.t)

    def turnDirection(self, e, direction):
        yaw = self.getYaw(e)
        if self.origin == 0:
            self.origin = yaw
        else:
            print 'Origin: ', self.origin
            print 'Yaw: ', yaw
            print -abs(yaw)+abs(self.origin)
            if abs(yaw - self.origin) < 3.1415/8:
                self.t.angular.z = 0.5
            else:
                self.t.angular.z = 0
                self.origin = 0
                self.turning = False
        
        self.rotator_pub.publish(self.t)


    def cb(self, e):

        if self.proximity == 0 and self.turning == False:
            self.t.angular.z = 0
            self.driveForward(e, 20, 2)
        else:
            self.turning = True
            if self.turning:
                self.t.linear.x = 0
                self.turnDirection(e, 1)
        




driver = Driver()
print 'Spinning'
rospy.spin()