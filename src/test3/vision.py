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
        
    
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.state_sub = rospy.Subscriber('/state', String, self.stateCB)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)

        self.state = "Drive"
        
        self.twist = Twist()
        self.target = 0


    #Legacy Code Unused
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
        #Sets state to state subscription
        self.state = state.data

    def FindTarget(self, mask, image):
        #Returns mean location of colours that fit the mask on the image
        M = cv2.moments(mask)
        return (M['m00'] > 0)

    def image_callback(self, data):

        namedWindow("Image window", 1)

        #Gets image from robot and set it to BGR
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        h, w, d = cv_image.shape

        #Crops CV image to ensure that all colours found is from directly in front of it
        #This is to make sure that the robot doesn't see red out of the corner of it's eye and get scared and not walk down an otherwise valid route
        cv_image = cv_image[380:400, 200:500]


        #Define Masks ranges in BGR
        bgr_blue_mask = inRange(cv_image,
                                 np.array((100, 0, 0)), #230
                                 np.array((255, 5, 5))) #255
                                 
        bgr_green_mask = inRange(cv_image,
                                 np.array((0, 100, 0)), #230
                                 np.array((5, 255, 5))) #255

        bgr_red_mask = inRange(cv_image,
                                 np.array((0, 0, 100)), #230
                                 np.array((5, 5, 255))) #255

        lower_yellow = np.array([0, 90, 90])
        upper_yellow = np.array([10, 255, 255])
        bgr_yellow_mask = inRange(cv_image, lower_yellow, upper_yellow)

        hsv_img = cvtColor(cv_image, COLOR_BGR2HSV)

        print (np.mean(hsv_img[:, :, 0]))
        print (np.mean(hsv_img[:, :, 1]))
        print (np.mean(hsv_img[:, :, 2]))
        print ('====')


        #Stop state to override, largely legacy code from iteration 1
        if self.state != "Stop":

            #If can see a green, blue, red, or yellow, set target state to ID of found - Largely legacy code from iteration 2
            if (self.FindTarget(bgr_green_mask, cv_image)): self.target = 0
            # elif (self.FindTarget(bgr_blue_mask, cv_image)): self.target = 1 ##LEGACY CODE
            elif (self.FindTarget(bgr_red_mask, cv_image)): self.target = 2
            # elif (self.FindTarget(bgr_yellow_mask, cv_image)): self.target = 3 ##LEGACY CODE
            else: self.target = -1

            #If a target has been found, do something with it: Set State to colour
            #ImageMoments beelines directly to target

            if self.target == 0:
                # self.ImageMoments(bgr_green_mask, cv_image, w) ##Legacy Code
                if self.state != "Green": self.state_pub.publish(String(data="Green"))
            elif self.target == 1:
                self.ImageMoments(bgr_blue_mask, cv_image, w)
                if self.state != "Blue": self.state_pub.publish(String(data="Blue"))
            elif self.target == 2:
                if self.state != "Red": self.state_pub.publish(String(data="Red"))
            elif self.target == 3:
                self.ImageMoments(bgr_yellow_mask, cv_image, w)
                if self.state != "Yellow": self.state_pub.publish(String(data="Yellow"))
            else:
                print'Nothing Found'
                if self.state != "Drive": self.state_pub.publish(String(data="Drive"))
        

        imshow("Image window", cv_image) #Show, in a window, the image
        waitKey(1)


startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()
