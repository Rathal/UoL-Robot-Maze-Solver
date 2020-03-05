import rospy
from std_msgs.msg import String, Float64
import cv2
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny
from cv2 import inRange, cvtColor, COLOR_BGR2HSV, findContours, RETR_TREE, CHAIN_APPROX_SIMPLE, contourArea, drawContours
from sensor_msgs.msg import Image
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
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        
        self.twist = Twist()
        self.target = False
        self.isBlue = False

    def ImageMoments(self, mask, image, w):
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print('cx: %f, cy: %f' %(cx, cy))
            cv2.circle(image, (cx, cy), 20, (255, 0, 0), -1)
            err = cx - w/2
            #self.blueError_pub.publish(err)
            self.twist.linear.x = 0.1
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            print err

    def image_callback(self, data):
        # namedWindow("Image window")
        # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # imshow("Image window", cv_image)
        # waitKey(1)

        namedWindow("Image window", 1)
        #namedWindow("Yellow window", 1)

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        h, w, d = cv_image.shape

        bgr_blue_mask = inRange(cv_image,
                                 np.array((100, 0, 0)), #230
                                 np.array((255, 5, 5))) #255
                                 

        lower_yellow = np.array([40,100,80])
        upper_yellow = np.array([80,255,250])
        bgr_yellow_mask = inRange(cv_image, lower_yellow, upper_yellow)

        hsv_img = cvtColor(cv_image, COLOR_BGR2HSV)
        # hsv_thresh = inRange(hsv_img,
        #                          np.array((0, 150, 50)),
        #                          np.array((255, 255, 255)))

        print (np.mean(hsv_img[:, :, 0]))
        print (np.mean(hsv_img[:, :, 1]))
        print (np.mean(hsv_img[:, :, 2]))
        print ('====')

        #cv2.bitwise_and(cv_image, cv_image, mask=bgr_blue_mask)

        if (self.target == False): #While Blue Not Found (WIP)OR SPUN FOR TOO LONG
            print 'Looking for Blue'
            M = cv2.moments(bgr_blue_mask)
            if M['m00'] > 0:
                self.target = True
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print('cx: %f, cy: %f' %(cx, cy))
                cv2.circle(cv_image, (cx, cy), 20, (255, 0, 0), -1)
                err = cx - w/2
                print 'Blue Found'
            else:
                #self.twist.linear.x = 0.1
                self.twist.angular.z = 0.2
                self.cmd_vel_pub.publish(self.twist)
        print 'Target Spotted'
        self.ImageMoments(bgr_blue_mask, cv_image, w)


        # if (self.isBlue):
        #     self.ImageMoments(bgr_yellow_mask, cv_image, w)
        # else:
        #     self.ImageMoments(bgr_blue_mask, cv_image, w)
        

        imshow("Image window", cv_image) #bgr_blue_mask
        waitKey(1)


startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()
