#!/usr/bin/env/python

"""
CMP3103M Autonomous Mobile Robotics
Assessment Item 1

@author: James McArthur
ID: 06094508
"""

import rospy, cv2, cv_bridge, numpy, math, actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from numpy import nanmean, nanmin
from kobuki_msgs.msg import BumperEvent
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class ColourChaser:

    def __init__(self):

       
        cv2.startWindowThread()

        self.bridge = cv_bridge.CvBridge()
        
        #Determine speed of robot       
        self.linear_vel = 0.8
        
        self.twist = Twist()
        
        #===============================Subscribers=========================================
        self.raw_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.raw_cb)
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        self.bump_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bump_cb) #NEEDS bump callback
        
        #===============================Publishers=========================================
        
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
        

        #===============================Colour List=========================================   

        #Used to iterate through self.colours
        self.colour_index = 0 
        
        self.colours = ['Red', 'Green', 'Blue', 'Yellow']
        
        self.haveFound = {'Red':False, 'Green':False, 'Blue':False, 'Yellow':False}
        
        
        #===============================Waypoints========================================= 
        
        self.points = [(0.22,4.49),(1.50,-1.07),(-4.32,1.35)] 

        #Used to iterate through self.points
        self.point_index = 0 

        #Flag determines if randomly searching or moving to new position
        self.searching = True 
        

    def raw_cb(self, data):        

            #Raw image window
            cv2.namedWindow("window", 1)
            
            #Thresholded image window
            cv2.namedWindow("mask", 2)
            
            try:           
                cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
            
            except cv_bridge.CvBridgeError, e:
                print(e)
            
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)      
    
            #Select next colour to threshold
            mask = self.get_colour(hsv)
    
            width = cv_image.shape[1]
    
            M = cv2.moments(mask)               

            #If no colour has been found yet, keep looking
            if self.searching == True:
                
                if nanmin(self.laser > 0.75):

                    #Apply circle to objects center of mass and move towards it
                    if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
        
                        error = cx - width/2 

                        #Adjust regarding the difference between center of image and the centroid
                        self.explore(self.linear_vel, -float(error) / 100)
        
                        if(self.center < 1.25) and (self.center >= 0.75):

                            #Register colour as found, switch to next colour and mark search flag as false
                            self.found()

                            #Check if all colours have been found
                            self.colour_check()
        
                    else:
                        self.change_colour()
                        
                        if nanmin(self.laser) < 1 and (math.isnan(self.center) == False):

                            #Avoid obstacles directly in front of turtlebot
                            self.avoid()                       
                                           
                        else:
                            #Otherwise keep moving forward
                            self.explore(self.linear_vel, 0)                                          
                else:
                    self.explore(-1, 0)
            else:
                #Move to one of three different waypoints
                self.go_to_waypoint(self.points[self.point_index])              
                
            cv2.imshow("window", cv_image)
            cv2.waitKey(1)
            
            cv2.imshow("mask", mask)
            cv2.waitKey(1)
        
                   
    #Get ranges for the laser (left, right and center)                  
    def scan_cb(self, data):
                          
        self.laser = data.ranges
        
        self.center = data.ranges[len(data.ranges) / 2]
        
        self.right = self.laser[0]
        
        self.left = self.laser[len(self.laser)-1]
    
    #Reverse if bumper is pressed
    def bump_cb(self,data):
        
        if(data.state == BumperEvent.PRESSED):
            
            print("oops! Think I hit something!")
            
            self.explore(-1, 1.5)                  
    
    #Move turtlebot based on linear and angular arguments
    def explore(self, v, a):
        
        self.twist.linear.x = v
        
        self.twist.angular.z = a
        
        self.velocity_pub.publish(self.twist)
     
    #If a colour is identified from dictionary, mark it as found and switch flag to goal pattern
    def found(self):
        
        if(self.colours[self.colour_index] == 'Red'):
            
            self.haveFound['Red'] = True
            
            self.change_colour()
            
            self.searching = False
            
            print("I've found Red!")

        elif(self.colours[self.colour_index] == 'Green'):
            
            self.haveFound['Green'] = True
            
            self.change_colour()
            
            self.searching = False
            
            print("Found a green object!")

        elif(self.colours[self.colour_index] == 'Blue'):
            
            self.haveFound['Blue'] = True
            
            self.change_colour()
            
            self.searching = False
            
            print("There's Blue!")

        elif(self.colours[self.colour_index] == 'Yellow'):
            
            self.haveFound['Yellow'] = True
            
            self.change_colour()
            
            self.searching = False
            
            print("Gotcha Yellow!")
            
    
    #Changes colour on each callback until corresponding colour is found
    def change_colour(self):
        
        if self.colour_index < 3:
            
            self.colour_index = self.colour_index + 1
            
        else:
            self.colour_index = 0             
                     
    #Avoid anything that falls inside condition statements
    def avoid(self):

        if (math.isnan(nanmean(self.laser)) == False):
                
            if nanmean(self.right) < 0.75:
                self.explore(0,1.2)  
       
            elif nanmean(self.left) < 0.75:
                self.explore(0,-1.2)
                
            elif math.isnan(self.left) == True and math.isnan(self.center) == True and math.isnan(self.right) == True:
            
                self.explore(-1, 1)
                
            else:
                self.explore(0, -1.2)                
                
        else:
            self.explore(-1, 0.5)
    
    #set new colour bounds using colour model (depending on index value)
    def get_colour(self, colourModel):

        if(self.colours[self.colour_index] == 'Red'\
        and self.haveFound['Red'] == False):
            lower_bound = numpy.array([ 0, 200, 100])
            upper_bound = numpy.array([ 0, 255, 150])
                     
        elif(self.colours[self.colour_index] == 'Green'\
        and self.haveFound['Green'] == False):
            lower_bound = numpy.array([ 40, 100,  80])
            upper_bound = numpy.array([ 80, 255, 250])

        elif(self.colours[self.colour_index] == 'Blue'\
        and self.haveFound['Blue'] == False):
            lower_bound = numpy.array([ 100, 100,  100])
            upper_bound = numpy.array([ 130, 255, 255])

        elif(self.colours[self.colour_index] == 'Yellow'\
        and self.haveFound['Yellow'] == False):
            lower_bound = numpy.array([ 30, 200,  100])
            upper_bound = numpy.array([ 50, 255, 150])
        #set to white
        else:
            lower_bound = numpy.array([ 255, 255, 255])
            upper_bound = numpy.array([ 255, 255, 255])

        return cv2.inRange(colourModel, lower_bound, upper_bound)
    
    #Defines a new action to move to a new waypoint using SimpleActionClient
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
        
        #If turtlebot navigates successfully, update waypoint and start searching for colours       
        if move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:         
            print("Made it to the goal!")
            self.check_waypoint()
            self.searching = True
            
    #updates waypoint (resets if greater than array length)                        
    def check_waypoint(self):
        
        if self.point_index <= 1:
            
            self.point_index = self.point_index + 1
                  
        else:
            self.point_index = 0
    
    #Checks if all colours have been found
    def colour_check(self):
        
        if self.haveFound['Red'] == True\
        and self.haveFound['Green'] == True\
        and self.haveFound['Blue'] == True\
        and self.haveFound['Yellow'] == True:

            #If all found, end the program
            print("I've found all the colours!")
            rospy.signal_shutdown("Program Completed")
            
            
       

#Instantiate and loop chaser object               
def main():
       
    rospy.init_node('chaser')
    chaser = ColourChaser()      
    rospy.spin()           
    
    
if __name__=="__main__":
        main()
