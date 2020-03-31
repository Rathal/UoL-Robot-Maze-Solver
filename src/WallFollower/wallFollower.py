import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
#from kobuki_msgs.msg import BumperEvent

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, ceil

class Proximity:
    def __init__(self):
        rospy.init_node('Proximity')
        self.driver_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odomCB)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

        self.t = Twist()
        self.proximity = 0
        self.right = True
        self.foundWall = False
        self.straying = False
        self.goal = 0

        self.yaw = 0
        self.dYaw = 0

    def Chunk(self, list, n):
        for i in xrange(0, len(list), n):
            yield list[i:i+n]
        
    def DetermineProximity(self, distance, Ranges):
        if any(r < distance for r in Ranges[0]):
            print 'Too Close Right'
            self.proximity = 1
        elif any(r < distance for r in Ranges[1]):
            print 'Too Close Left'
            self.proximity = -1
        else: self.proximity = 0

    def Degree(self, radian):
        degree = radian*180/pi
        return degree
    
    def Radian(self, degree):
        radian = degree*pi /180
        return radian

    def getClosestCardinal(self, angle):        
        #Gets the closest cardinal direction to current yaw rate
        if pi/4 < angle <  3*pi/4: cardinal = pi/2
        elif 3*pi/4 < angle < 5*pi/4: cardinal = pi
        elif 5*pi/4 < angle < 7*pi/4: cardinal = 3*pi/2
        # 0 = 2pi, but different rotations, so seperated
        elif 0 < angle < pi/4: cardinal = 0
        elif 7*pi/4 < angle < 2*pi: cardinal = 2*pi
        print 'Cardinal: ', cardinal
        
        return cardinal

    def OdomToYaw(self, odom):
        ori = odom.pose.pose.orientation
        ori_list = [ori.x, ori.y, ori.z, ori.w]
        #Calculate roll, pitch, yaw, from orientation
        (r, p, y) = euler_from_quaternion(ori_list)            
        self.yaw = y+pi #Normalise for positive radians
        self.dYaw = self.Degree(self.yaw)

    def StayTrue(self):
        heading = self.getClosestCardinal(self.yaw)
        print 'Staying True to: ', heading
        print 'Current Yaw: ', self.yaw
        if self.yaw < heading-0.035:
            self.t.angular.z = 0.1
            self.driver_pub.publish(self.t)
        elif self.yaw > heading+0.035:
            self.t.angular.z = -0.1
            self.driver_pub.publish(self.t)
        
    def CycleRadian(self, radian):
        if radian > 2*pi:
            radian -= 2*pi
        return radian

    def turnDegree(self, degree):
        radian = self.Radian(degree+180)
        radian = self.CycleRadian(radian)
        radian += self.yaw
        radian = self.CycleRadian(radian)
        print 'Degrees: ', degree, ' Pos: ', self.Degree(self.yaw), ' Goal :', self.Degree(radian)
        self.faceAngle(radian)
    
    def faceAngle(self, angle):
        if ceil(self.Degree(angle)) == 360:angle = 0
        print 'Angle: ', angle

        self.goal = angle
        #if round(self.goal, 1) == round(yaw, 1):
        if self.goal == 0:
            print 'Done'
            self.t.angular.z = 0
            # self.state_pub.publish(String(data="Drive"))
        elif self.goal >  self.yaw:
            print 'Left'
            self.t.angular.z = 0.25
        elif self.goal < self.yaw:
            print 'Right'
            self.t.angular.z = -0.25
        self.t.linear.x = 0
        
        self.driver_pub.publish(self.t)

    def odomCB(self, odom):
        self.OdomToYaw(odom)
        if self.foundWall:
            self.turnDegree(180)


    def GoForward(self, speed):
        print 'Driving'
        self.t.linear.x = speed
        self.StayTrue()
        self.driver_pub.publish(self.t)

    def FindWall(self, ranges):
        if any(r < 0.5 for r in ranges):
            self.foundWall = True
            print 'Wall Found'
        else:
            self.GoForward(0.1)

    def laser_cb(self, laser_msg):
        #Find Wall
        ranges = laser_msg.ranges
        if (self.foundWall == False):
            self.FindWall(ranges)




proximity = Proximity()
rospy.spin()