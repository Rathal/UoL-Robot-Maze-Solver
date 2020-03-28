import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Searcher:
    def __init__(self):
        rospy.init_node('Searcher')
        self.state_sub = rospy.Subscriber('/state', String, self.stateCB)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)
        
        self.rotator_sub = rospy.Subscriber('/odom', Odometry, callback=self.odomCB)
        self.spinner_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.state = "Null"
        self.t = Twist()

        self.spin = False
        self.spins = 0
        
        self.origin = -1
        self.goal = -1
    
    def stateCB(self, state):
        print "updating state"
        self.state = state.data
        print state
                    
        if self.state == 'Start Spin':
            self.spin = True
            self.spins = 0
            self.state_pub.publish(String(data="Spin"))
        if self.spin and self.state == "Update Spin":
            self.spins += 1
            self.state_pub.publish(String(data="Spin"))
            
            

    def odomCB(self, odom):
        from math import pi, ceil

        def degree(x):
            degree = x*180/pi
            return degree
        
        def cycleRadian(radian):
            if radian > 2*pi:
                radian -= 2*pi
            return radian

        def getClosestCardinal(angle):
            # cardinal = ((angle+(pi/4))//(pi/4))
            # cardinal -= 1
            # cardinal = cardinal * pi/2
            # if cardinal > 2*pi:
            #     cardinal -= 2*pi            
            if pi/4 < angle <  3*pi/4: cardinal = pi/2
            elif 3*pi/4 < angle < 5*pi/4: cardinal = pi
            elif 5*pi/4 < angle < 7*pi/4: cardinal = 3*pi/2
            else: cardinal = 0
            print 'Cardinal: ', cardinal
            
            return cardinal
        
        def faceAngle(angle, yaw):
            
            print 'Angle: ', angle
            if ceil(degree(angle)) == 360: angle = 0
            print 'Angle: ', angle

            self.goal = angle
            #if round(self.goal, 1) == round(yaw, 1):
            if self.goal == 0:
                self.t.angular.z = 0
                self.state_pub.publish(String(data="Waiting"))
            elif self.goal >  yaw:
                print 'Left'
                self.t.angular.z = 0.25
            elif self.goal < yaw:
                print 'Right'
                self.t.angular.z = -0.25
            
            self.spinner_pub.publish(self.t)

        def turnDegree(degree, yaw):
            radian = degree*pi /180
            radian += yaw
            radian = cycleRadian(radian)
            print 'Degrees: ', degree, ' Pos: ', yaw*180/pi, ' Goal :', radian*180/pi
            faceAngle(radian, yaw)

        ori = odom.pose.pose.orientation
        ori_list = [ori.x, ori.y, ori.z, ori.w]
        #Calculate roll, pitch, yaw, from orientation
        (r, p, y) = euler_from_quaternion(ori_list)            
        yaw = y+pi

        if self.state == 'Initialised':
            # faceAngle(3*pi/2, yaw)
            print 'Init: Asking to Spin'
            self.state_pub.publish(String(data="req_Start Spin"))

        elif self.state == 'Turn To 90':
            print 'Turning to: E'
            turnDegree(90, yaw) ##0.1 or 359.9 for North
        elif self.state == 'Turn To 180':
            print 'Turning to: S'
            turnDegree(180, yaw) ##0.1 or 359.9 for North
        elif self.state == 'Turn To 270':
            print 'Turning to: W'
            turnDegree(270, yaw) ##0.1 or 359.9 for North
        elif self.state == 'Turn To 360':
            print 'Turning to: N'
            turnDegree(359.9, yaw) ##0.1 or 359.9 for North
        
        if self.state == "Spin":
            print 'Spin No: ', self.spins
            if self.spins == 0:
                self.state_pub.publish(String(data="req_Turn To 90"))
            elif self.spins == 1:
                self.state_pub.publish(String(data="req_Turn To 180"))
            elif self.spins == 2:
                self.state_pub.publish(String(data="req_Turn To 270"))
            elif self.spins == 3:
                self.state_pub.publish(String(data="req_Turn To 360"))
            


        
        # if self.state == 'Initialised':
        #     print 'Initialising'
        #     print 'Yaw: ', round(yaw, 2), ' Goal: ', self.goal
        #     self.goal = 0
        #     if round(yaw, 2) != round(self.goal, 2):
        #         self.t.angular.z = 0.1
        #     else:
        #         self.t.angular.z = 0
        #         self.goal = -1
        #         self.state_pub.publish(String(data="req_Spinning"))
        #     self.spinner_pub.publish(self.t)
        
        # if self.state == 'Spinning':
        #     print 'Yaw: ', yaw, ' Goal: ', round(self.goal, 2)

        #     if self.origin == -1:
        #         self.origin = yaw
        #         self.goal = yaw + (pi/4) 
        #         self.goal += 0.5
        #         if self.goal > 2*pi:
        #             self.goal -= (2*pi)
        #         print 'Origin: ', self.origin
        #         print 'Goal: ', self.goal

        #     if round(yaw, 2) != round(self.goal, 2):
        #         self.t.angular.z = 0.5
        #     else:
        #         self.t.angular.z = 0
        #         self.goal = -1
        #         self.state_pub.publish(String(data="Adjusting"))
        #     self.spinner_pub.publish(self.t)
        
        # if self.state == 'Adjusting':
        #     if self.goal == -1:
        #         self.goal = getClosestCardinal(y+pi)
        #     self.goal = 0
        #     print 'Goal: ', round(self.goal,5), ' Facing: ', round(yaw,5)

        #     if round(self.goal, 2) == round(yaw, 2):
        #         self.t.angular.z = 0
        #         self.state_pub.publish(String(data="Waiting"))
        #     elif self.goal >  yaw:
        #         print 'Bigger'
        #         self.t.angular.z = 0.05
        #     elif self.goal < yaw:
        #         print 'Smaller'
        #         self.t.angular.z = -0.05
            
        #     self.spinner_pub.publish(self.t)
        
searcher = Searcher()
rospy.spin()