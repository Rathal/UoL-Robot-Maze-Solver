import rospy
from std_msgs.msg import String, Float64, String
import cv2
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny
from cv2 import inRange, cvtColor, COLOR_BGR2HSV, findContours, RETR_TREE, CHAIN_APPROX_SIMPLE, contourArea, drawContours
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Twist


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.image_callback)
        #self.image_pub = rospy.Publisher('/result_topic', String)
        #self.blueError_pub = rospy.Publisher('/error/blue', Float64, queue_size=1)
        # self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.state_sub = rospy.Subscriber('/state', String, self.stateCB)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)

        self.state = "Drive"
        
        self.twist = Twist()
        self.target = 0
        self.isBlue = False

        self.init = True

    def ImageMoments(self, mask, image, w):
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print('cx: %f, cy: %f' %(cx, cy))
            cv2.circle(image, (cx, cy), 20, (255, 0, 0), -1)
            err = cx - w/2
            # self.blueError_pub.publish(err)
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            print err

    def stateCB(self, state):
        self.state = state.data

    def FindTargert(self, mask, image):
        M = cv2.moments(mask)
        return (M['m00'] > 0)

    def image_callback(self, data):
        # namedWindow("Image window")
        # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # imshow("Image window", cv_image)
        # waitKey(1)

        namedWindow("Image window", 1)
        #namedWindow("Yellow window", 1)

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        crop_image = cv_image[0:400, 0:640]
        gray = cv2.cvtColor(crop_image,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)

        lines = cv2.HoughLines(edges,1,np.pi/180,200)
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            cv2.line(crop_image,(x1,y1),(x2,y2),(0,0,255),2)

        imshow("Image window", crop_image) #bgr_blue_mask
        waitKey(1)


startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()
