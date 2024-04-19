#!/usr/bin/env python

#----------------------------------------------------------------
#   wallfollower.py 		Version 1.?
#
#   --- This module runs a continuous controller loop to provide correction
#   --- to a wall following rallycar. It controls distance and
#   --- parallelism to a wall.
#   ---
#   --- 04//2024 Initial coding. Copied from Voyles.
#   --- 04/	/2024 
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
    
    def __init__(self):
    # __init__(self)
    #
    # This function initializes all values for the wall follower class.
    #
    # self - the object created as part of the class, called with any class function
    #
      
        self.lwall_dist = 1.5 # LiDAR frame's distance from the left wall
        self.diagonal_dist = 1 # 
        self.front_dist = 10 # 
        self.rwall_dist = 1.5 # LiDAR frame's distance from the right wall
        self.des_dist = 1 # Desired distance, this is a constant for PD
        
        # velocities to send
        self.steer = 0.0
        self.gasvel = 0.0


        # TODO: Tune the gains below
        self.kp = 409.6
        self.kd = 2.048
        
        self.lwall_dist_prev = self.lwall_dist 
        self.rwall_dist_prev = self.rwall_dist 

        # TODO: Set initial velocity
        self.des_vel = Twist()
        self.des_vel.linear.x = 250     # m/s

        # Add variables to track rate of error
        self.err = 0
        self.prev_err = 0
        self.deltaT = 0
        self.err_rate = 0
        self.dq = deque([0,0,0,0,0])
        

    def Laser_cb(self, msg):
        # Laser_cb(self, msg)
        #
        # Laser_CB computes collects the lidar data from the point 90 DEG offset from
        # the +X axis facing left and the lidar data 45 DEG offset from the +X axis facing 
        # forward and to the left. It does so by collecting the indicies of the data points
        # then using those indicies to find the actual values from the lidar data.
        #
        # msg - Message data from the lidar node
        # self - the object created as part of the class, called with any class function
        
        rangeCount = len(msg.ranges) # Determine the amount of laser scans performed
        centerIndex = rangeCount // 2 #This is the data point that corresponds with the +X axis
        angleIncrement = msg.angle_increment * 180/np.pi #Convert increment data from rad to deg
        #Determin the amount of indicies to that correspond with 90 DEG offset and the 45 DEG offset
        indexOffset = round(90 / angleIncrement) 
        diagonalOffset = round(45 / angleIncrement)
        #Add index offsets to the center value to find left and diagonal indicies.
        leftIndex = centerIndex + indexOffset 
        diagonalIndex = centerIndex + diagonalOffset
    		#Find the value of each datapoint based on the indices
        self.lwall_dist = msg.ranges[leftIndex]
        self.diagonal_dist = msg.ranges[diagonalIndex]
        self.front_dist = msg.ranges[centerIndex]

    def Control_loop(self, freq=40):
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

      
        r = rospy.Rate(freq)
        self.deltaT = 1 / freq        
        rospy.Subscriber("scan", LaserScan, self.Laser_cb)
        self.accel_pub = rospy.Publisher('/accelerator_cmd', Float32, queue_size=10)
        self.steer_pub = rospy.Publisher('/steering_cmd', Float32, queue_size=10)
            
        while not rospy.is_shutdown():
        
            # Calculate error
            alpha = np.arctan((self.diagonal_dist * np.cos(np.pi / 4) - self.lwall_dist) / (self.diagonal_dist * np.sin(np.pi / 4)))
            AB = self.lwall_dist * np.cos(alpha)
            CD = AB + self.gasvel * self.deltaT * np.sin(alpha)
            print("CD:",CD)
            # Set error and error rates
            self.prev_err = self.err
            self.err = CD - self.des_dist # Set error based on distance from wall/angle
            self.dq.append(self.err)
            self.dq.popleft()
            self.err_rate = sum(self.dq)/(self.deltaT*5) 

            # Update the desired angular velocity using PD
            self.steer = (self.kp*self.err + self.kd*self.err_rate)
            print("KP Impact: ", self.kp * self.err)
            print("KD Impact: ", self.kd * self.err_rate)
            if (self.steer < -2047):
            	self.steer = -2047
            if (self.steer > 2047):
            	self.steer = 2047
            self.gasvel = self.des_vel.linear.x
            if (self.front_dist < 0.4):
                self.gasvel = 0
                 
                print("Too close")
                
            self.accel_pub.publish(Float32(self.gasvel))
            self.steer_pub.publish(Float32(self.steer))

            # Move robot
            r.sleep()

    def Clean_shutdown(self):
        accel_pub = rospy.Publisher("/accelerator_cmd", Float32, queue_size=10)
        steer_pub = rospy.Publisher("/steering_cmd", Float32, queue_size=10)
        
        steer_pub.publish(Float32(0))
        accel_pub.publish(Float32(0))

if __name__=="__main__":
    rospy.init_node("wall_follow", anonymous=True)

    try:
        x = WallFollower()
        x.Control_loop()
    except rospy.ROSInterruptException:
        x.Clean_shutdown()
        pass
