# CMP3103M Autonomous Mobile Robotics

## Colour Chaser

The goal of this project was to develop an object search behaviour, programmed in Python using ROS, that enables a
robot to search for coloured objects visible in the robot’s camera. This assessment is purely done in
simulation (gazebo), and not on the real robot.

The python code implements a behaviour that enables the robot, in simulation, to find a total of 4 objects
distributed in a simulated environment. This uses a combination of the robot’s sensory input and its actuators
to guide the robot to each item. Success in locating an item is defined as: (a) being less than 1m from
the item, and (b) indication from the robot that it has found an object. 

The simulated environment includes four brightly coloured objects hidden in the environment at
increasing difficulty. the turtlebot starts from a predefined position.  

The robots design includes the following elements:

  1. Perception of the robot’s environment using the Kinect sensor, using both RGB Raw and laserScan topics
     data in order to find each object;
     
  2. An implementation of an appropriate control law implementing a search behaviour on the
     robot. This includes a random search pattern that uses laserscans to avoid obstacles in the environment,
     as well as the movebasegoal action from the ROS nav stack to move to new areas.

