#!/usr/bin/env python

#----------------------------------------------------------------
#   wallfollower.py 		Version 1.?
#
#   --- This module runs a continuous controller loop to provide correction
#   --- to a wall following rallycar. It controls distance and
#   --- parallelism to a wall.
#   ---
#   --- 04//2024 Initial coding. Copied from Voyles.
#   --- 04/ /2024 
#   --- 04/ /2024 
#---------------------------------------------------------------

import numpy as np
import rospy
import time
from collections import deque
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from std_msgs.msg import Float32


import serial

class WallFollower:
    
    # __init__(self)
    #
    # This function initializes all values for the wall follower class.
    #
    # self - the object created as part of the class, called with any class function
    def __init__(self):
      
        self.lwall_dist = 1.5 # LiDAR frame's distance from the left wall
        self.diagonal_dist = 1 # 
        self.front_dist = 10 # 
        self.rwall_dist = 1.5 # LiDAR frame's distance from the right wall
        self.des_dist = 1 # Desired distance, this is a constant for PD
        
        # Set initial control values
        self.steer = 0.0
        self.gasvel = 0.0

        # Tuned the gains
        self.kp = 409.6
        self.kd = 2.048
        
        self.lwall_dist_prev = self.lwall_dist 
        self.rwall_dist_prev = self.rwall_dist 

        # Set initial velocity
        self.des_vel = Twist()
        self.des_vel.linear.x = 250     # m/s

        # Variables to track rate of error
        self.err = 0
        self.prev_err = 0
        self.deltaT = 0
        self.err_rate = 0
        self.dq = deque([0,0,0,0,0])

    # Laser_cb(self, msg)
    #
    # Laser_CB computes collects the lidar data from the point 90 DEG offset from
    # the +X axis facing left and the lidar data 45 DEG offset from the +X axis facing 
    # forward and to the left. It does so by collecting the indicies of the data points
    # then using those indicies to find the actual values from the lidar data.
    #
    # msg - Message data from the lidar node
    # self - the object created as part of the class, called with any class function    
    def Laser_cb(self, msg):
        
        rangeCount = len(msg.ranges) # Determine the amount of laser scans performed by counting the amount of datapoints
        centerIndex = rangeCount // 2 # Data point that corresponds with the +X axis facing forward
        angleIncrement = msg.angle_increment * 180/np.pi # Convert provided increment from rad to deg
        # Determine the amount of indicies to that correspond with 90 DEG offset and the 45 DEG offset
        indexOffset = round(90 / angleIncrement) 
        diagonalOffset = round(45 / angleIncrement)
        # Add index offsets to the center value to find left and diagonal indicies.
        leftIndex = centerIndex + indexOffset 
        diagonalIndex = centerIndex + diagonalOffset
    		# Find the value of each datapoint based on the indices
        self.lwall_dist = msg.ranges[leftIndex]
        self.diagonal_dist = msg.ranges[diagonalIndex]
        self.front_dist = msg.ranges[centerIndex]
        
    # Control_loop(self, freq)   
    #
    # Runs a continuous loop at a frequency of 40 Hz. A subscriber is made to read distance data from
    # the LiDAR mounted on a rallycar. Two publishers are created to give feedback to the rallycar's
    # motor and control velocity. The value that controls the motor's forward velocity is then adjusted
    # based on the error between the desired distance from the wall and the LiDAR's reading. In order to
    # control the parallelism of the car to the wall, an equation which finds the error from a previous position of the car
    # to a current position of the car, measured by the distance perpendicular to the LiDAR and the distance
    # 45 degrees from the LiDAR. The distances can be used to find the current angle and compare it to the desired angle.
    #
    # freq - control loop frequency (Reccomended Range : [30,100])
    # self - the object created as part of the class, called with any class function
    def Control_loop(self, freq=40):
      
        r = rospy.Rate(freq)
        self.deltaT = 1 / freq
        # Declare subscriber and publisher nodes
        rospy.Subscriber("scan", LaserScan, self.Laser_cb) # Subscribe to scan topic with LaserScan data structure and stores the LiDAR data
        self.accel_pub = rospy.Publisher('/accelerator_cmd', Float32, queue_size=10) # Publish a command for controlling the x aka forward velocity
        self.steer_pub = rospy.Publisher('/steering_cmd', Float32, queue_size=10) # Publish a command for controlling the yaw aka steer velocity
            
        while not rospy.is_shutdown():
        
            # Calculate error using two lidar data points. (Source: Penn Engineering: PID Control For Wall Following)
            alpha = np.arctan((self.diagonal_dist * np.cos(np.pi / 4) - self.lwall_dist) / (self.diagonal_dist * np.sin(np.pi / 4)))
            AB = self.lwall_dist * np.cos(alpha) # Current distance from car to wall
            CD = AB + self.gasvel * self.deltaT * np.sin(alpha) # Future distance from car to wall based on trajectory
            # Set error and error rates
            self.prev_err = self.err
            self.err = CD - self.des_dist # Set error based on distance from wall/angle
            # Append current error to a list containing the last 5 errors then calculate change in error using these values.
            self.dq.append(self.err) 
            self.dq.popleft()
            self.err_rate = sum(self.dq)/(self.deltaT*5) 
            # Update the desired angular velocity using PD
            self.steer = (self.kp*self.err + self.kd*self.err_rate)
            print("KP Impact: ", self.kp * self.err)
            print("KD Impact: ", self.kd * self.err_rate)
            # If calculated value exceeds maximum or minimum steering value, clip value to fit within range of steering values.
            if (self.steer < -2047):
            	self.steer = -2047
            if (self.steer > 2047):
            	self.steer = 2047
            self.gasvel = self.des_vel.linear.x
            # Check if car is getting too close to the wall. If so, stop the car.
            if (self.front_dist < 0.4):
                self.gasvel = 0
            self.accel_pub.publish(Float32(self.gasvel)) # Publish the new forward velocity value
            self.steer_pub.publish(Float32(self.steer)) # Publish the new steering velocity value
            # Move robot
            r.sleep()

    # def Clean_shutdown(self)
    #
    # Shutdown function that ensures both accelerator and steering values are
    # set to zero prior to shutdown of the program.
    #
    #self - the object created as part of the class, called with any class function
    def Clean_shutdown(self):
        
        accel_pub = rospy.Publisher("/accelerator_cmd", Float32, queue_size=10) # Creating a publisher for the forward velocity
        steer_pub = rospy.Publisher("/steering_cmd", Float32, queue_size=10) # Creating a publisher for the steering velocity
        
        steer_pub.publish(Float32(0)) # Publishing 0 to the steering velocity in order to stop steering
        accel_pub.publish(Float32(0)) # Publishing 0 to the forward velocity in order to stop forward velocity
        
# Main control loop.
#
# Checks for an intterupt from ros before executing shutdown.
if __name__=="__main__":

    rospy.init_node("wall_follow", anonymous=True) # Initiaing a node for the wall follower
    try:
        car8 = WallFollower() # Creating car8 as part of the wall follower
        car8.Control_loop() # Calling the control loop to start wall following 
    except rospy.ROSInterruptException:
        car8.Clean_shutdown()
        pass
