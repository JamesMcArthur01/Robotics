#!/usr/bin/env/python

import rospy, cv2, cv_bridge, numpy, math, actionlib, time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from numpy import nanmean, nanmin, nansum
from kobuki_msgs.msg import BumperEvent
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
        
        self.bump_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, bump_cb) #NEEDS bump callback
        
        #===============================Publishers=========================================
        
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
        

        #===============================Colour List=========================================   
    
        self.colour_index = 0
        
        self.colours = ['Red', 'Green', 'Blue', 'Yellow']
        
        self.haveFound = {'Red':False, 'Green':False, 'Blue':False, 'Yellow':False}
        
        
        #===============================Waypoints========================================= 
        
        self.points = [(0.22,4.49),(1.50,-1.07),(-4.32,1.35)] 
        
        self.point_index = 0
        
        self.search_timer = time.time() + 20
        

    def raw_cb(self, data):
        
        #if time.time() < self.search_timer:
            #Raw image window
            cv2.namedWindow("window", 1)
            #Thresholded image window
            cv2.namedWindow("mask", 2)
            
            try:
            #Converted from ROS image (data) to OpenCv image (BGR)
                cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
            
            except cv_bridge.CvBridgeError, e:
                print(e)
            #Change colour model from BGR to HSV 
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)      
    
            #Choose a colour to threshold
            mask = self.get_colour(hsv)
    
            width = cv_image.shape[1]
    
            M = cv2.moments(mask) #centroid            
    
            if time.time() < self.search_timer:
                
                if nanmin(self.laserArray > 0.75):
                    
                    if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
        
                        error = cx - width/2 #difference between the center of the image and the centroid
                        
                        self.explore(self.linear_vel, -float(error) / 100)
        
                        if(self.currentDist < 1.25) and (self.currentDist >= 0.75):
                            
                            self.found()
        
                    else:
                        self.change_colour()
                        
                        if nanmin(self.laserArray) < 1 and (math.isnan(self.currentDist) == False):
                            
                            self.avoid()
                                           
                        else:
                            self.explore(self.linear_vel, 0)
                                             
                else:
                    self.explore(-1, 0)
            else:
                self.go_to_waypoint(self.points[self.point_index])
            
            cv2.imshow("window", cv_image)
            cv2.waitKey(1)
            
            cv2.imshow("mask", mask)
            cv2.waitKey(1)
        
                   
                      
    def scan_cb(self, data):
              
        self.currentDist = data.ranges[len(data.ranges) / 2]
        
        self.laserArray = data.ranges
        
        self.right = self.laserArray[0]
        
        self.left = self.laserArray[len(self.laserArray)-1]              
        
        
    def explore(self, v, a):
        
        self.twist.linear.x = v
        
        self.twist.angular.z = a
        
        self.velocity_pub.publish(self.twist)
        
    def found(self):
        
        if(self.colours[self.colour_index] == 'Red'):
            
            self.haveFound['Red'] = True
            
            self.change_colour()
            
            print("I've found Red!")

        elif(self.colours[self.colour_index] == 'Green'):
            
            self.haveFound['Green'] = True
            
            self.change_colour()
            
            print("Found a green object!")

        elif(self.colours[self.colour_index] == 'Blue'):
            
            self.haveFound['Blue'] = True
            
            self.change_colour()
            
            print("There's Blue!")

        elif(self.colours[self.colour_index] == 'Yellow'):
            
            self.haveFound['Yellow'] = True
            
            self.change_colour()
            
            print("Gotcha Yellow!")

    def change_colour(self):
        
        if self.colour_index < 3:
            
            self.colour_index = self.colour_index + 1
            
        else:
            self.colour_index = 0  
                     
    
    def avoid(self):

        if (math.isnan(nanmean(self.laserArray)) == False):
            
            if nanmean(self.currentDist) < 1.5 or nanmin(self.currentDist) < 1:
                
                self.explore(0, -1.2)
                
                print("Front blocked - Turning Right")

            elif nansum(self.left) < nansum(self.right):
                
                self.explore(0.3, 1)
                
                print("Right Blocked - Going Left")
                
            elif nansum(self.left) > nansum(self.right):
                
                self.explore(0.3, -1)
                
                print("Left Blocked - Going Right")
        else:
            self.explore(-1, 0.5)

    def get_colour(self, colourModel):

        if(self.colours[self.colour_index] == 'Red' and self.haveFound['Red'] == False):
            lower_bound = numpy.array([ 0, 200, 100])
            upper_bound = numpy.array([ 0, 255, 150])
                     
        elif(self.colours[self.colour_index] == 'Green' and self.haveFound['Green'] == False):
            lower_bound = numpy.array([ 40, 100,  80])
            upper_bound = numpy.array([ 80, 255, 250])

        elif(self.colours[self.colour_index] == 'Blue' and self.haveFound['Blue'] == False):
            lower_bound = numpy.array([ 100, 100,  100])
            upper_bound = numpy.array([ 130, 255, 255])

        elif(self.colours[self.colour_index] == 'Yellow' and self.haveFound['Yellow'] == False):
            lower_bound = numpy.array([ 30, 200,  100])
            upper_bound = numpy.array([ 50, 255, 150])
        #white
        else:
            lower_bound = numpy.array([ 255, 255, 255])
            upper_bound = numpy.array([ 255, 255, 255])

        return cv2.inRange(colourModel, lower_bound, upper_bound)
    
    def go_to_waypoint(self, wp):
        
        move_client = actionlib.SimpleActionClient('/move_base/', MoveBaseAction)
        
        while(not move_client.wait_for_server(rospy.Duration.from_sec(5.0))):
                   rospy.loginfo("Waiting for the move_base action server to come up")               
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp[0]
        goal.target_pose.pose.position.y = wp[1]
        
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.z = -1.0
        
        
        move_client.send_goal(goal)
        move_client.wait_for_result(rospy.Duration.from_sec(5.0))
        
        if move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print("WAHOO!")
            self.point_index = self.point_index + 1
            self.search_timer = time.time() + 20
           
def main():
       
    rospy.init_node('chaser')
    chaser = ColourChaser()      
    rospy.spin()           
    
    
if __name__=="__main__":
        main()
