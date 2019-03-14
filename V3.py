#!/usr/bin/env/python


import rospy, cv2, cv_bridge, numpy, math, actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from math import radians
from numpy import nanmean, nanmin, nansum
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class ColourChaser:

    def __init__(self):

        #Start an image display thread
        cv2.startWindowThread()

        #OpenCV bridge object to convert ROS images into OpenCV images
        self.bridge = cv_bridge.CvBridge()
        
        #Determine speed of robot       
        self.linear_vel = 0.8
        
        #Instance of Twist message
        self.twist = Twist()
        
        #===============================Subscribers=========================================
        self.raw_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.raw_cb)
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        #===============================Publishers=========================================
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
        

        #===============================Colour List=========================================       
        self.index = 0
        
        self.colours = ['Yellow', 'Green', 'Blue', 'Red']
        
        self.haveFound = {'Yellow':False, 'Green':False, 'Blue':False, 'Red':False}
                    

    def raw_cb(self, data):

        #Raw image window
        cv2.namedWindow("window", 1)
        #Thresholded image window
        cv2.namedWindow("mask", 2)

        #Converted from ROS image (data) to OpenCv image (BGR)
        cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')

        #Change colour model from BGR to HSV 
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)      

        #Choose a colour to threshold
        mask = get_colour(hsv)

        width = cv_image.shape[1]

        M = cv2.moments(mask) #centroid

        if nanmin(self.laserArray > 0.75):
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)

                err = cx - w/2
                
                self.search(self.linear_vel, -float(err) / 100)

                if(self.currentDist < 1.25) and (self.currentDist >= 0.75):
                    
                    self.found()

            else:
                self.index += self.index+1
                
                if nanmin(self.laserArray) < 1 and (math.isnan(self.currentDist) == False):
                    
                    self.avoid()
                    
                else:
                    self.search(self.linear_vel, 0)
                
                if(self.index > 3):
                    
                    self.index = 0
        else:
            self.search(-1, 0)



        cv2.imshow("window", image)
        cv2.waitKey(1)
        
        cv2.imshow("mask", mask)
        cv2.waitKey(1)


    def scan_cb(self, data):
        
        self.currentDist = data.ranges[len(data.ranges) / 2]
        
        self.laserArray = data.ranges
        
        self.leftQuart = len(self.laserArray) / 4
        
        self.rightQuart = self.leftQuart * 3
        
    def search(self, v, a):
        
        self.twist.linear.x = v
        
        self.twist.angular.z = a
        
        self.velocity_pub.publish(self.twist)
        
    def found(self):
        
        if(self.colours[self.index] == 'Yellow'):
            
            self.haveFound['Yellow'] = True
            
            self.index += self.index+1
            
            print("============== Yellow Found ==============")

        elif(self.colours[self.index] == 'Green'):
            
            self.haveFound['Green'] = True
            
            self.index += self.index+1
            
            print("============== Green Found ==============")

        elif(self.colours[self.index] == 'Blue'):
            
            self.haveFound['Blue'] = True
            
            self.index += self.index+1
            
            print("============== Blue Found ==============")

        elif(self.colours[self.index] == 'Red'):
            
            self.haveFound['Red'] = True
            
            self.index += self.index + 1
            
            print("============== Red Found ==============")
            
        if(self.index > 3):
            
            self.index = 0
            
    
    def avoid(self):

        if (math.isnan(nanmean(self.laserArray)) == False):
            
            if nanmean(self.laserArray) < 1.5 or nanmin(self.laserArray) < 1:
                
                self.search(0, -1)
                
                print("Front blocked - Turning Right")

            elif nansum(self.laserArray[:self.leftQuart]) < nansum(self.laserArray[self.rightQuart:]):
                
                self.search(0.3, 1)
                
                print("Right Blocked - Going Left")
                
            elif nansum(self.laserArray[:self.leftQuart]) > nansum(self.laserArray[self.rightQuart:]):
                
                self.search(0.3, -1)
                
                print("Left Blocked - Going Right")
        else:
            self.search(-1, 0.5)

    def get_colour(colourModel):

        if(self.colours[self.index] == 'Yellow' and self.haveFound['Yellow'] == False):
            lower_bound = numpy.array([ 30, 200,  100])
            upper_bound = numpy.array([ 50, 255, 150])
            
        elif(self.colours[self.index] == 'Green' and self.haveFound['Green'] == False):
            lower_bound = numpy.array([ 40, 100,  80])
            upper_bound = numpy.array([ 80, 255, 250])

        elif(self.colours[self.index] == 'Blue' and self.haveFound['Blue'] == False):
            lower_bound = numpy.array([ 100, 100,  100])
            upper_bound = numpy.array([ 130, 255, 255])

        elif(self.colours[self.index] == 'Red' and self.haveFound['Red'] == False):
            lower_bound = numpy.array([ 0, 200, 100])
            upper_bound = numpy.array([ 0, 255, 150])
        #white
        else:
            lower_bound = numpy.array([ 255, 255, 255])
            upper_bound = numpy.array([ 255, 255, 255])

        return cv2.inRange(colourModel, lower_bound, upper_bound)
        


rospy.init_node('chaser')
chaser = ColourChaser()
rospy.spin()
