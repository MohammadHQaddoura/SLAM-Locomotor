#!/usr/bin/env python

#----------------------------------------------------------------
#   File to perform wall following using PD control
#
#   TODO: Complete the desired sections and
#           fix any issues in the code
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
        self.lwall_dist = 1.5
        self.diagonal_dist = 1
        self.front_dist = 10
        self.rwall_dist = 1.5
        self.des_dist = 1
        
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
        

    def laser_cb(self, msg):
        # Compute the distance to left and right walls
        #   using the data in msg. Does not use any hard-coded values       
        rangeCount = len(msg.ranges) #1081
        #print("range count: ", rangeCount)
        centerIndex = rangeCount // 2
        angleIncrement = msg.angle_increment * 180/np.pi
        # print(angleIncrement)
        indexOffset = round(90 / angleIncrement)
        # print(indexOffset)
        diagonalOffset = round(45 / angleIncrement)
        # print(diagonalOffset)
        
        leftIndex = centerIndex + indexOffset
        diagonalIndex = centerIndex + diagonalOffset
        #rightIndex = centerIndex - indexOffset
        # print(rightIndex)
    
    # These values are computed in control_loop
        self.lwall_dist = msg.ranges[leftIndex]
        self.diagonal_dist = msg.ranges[diagonalIndex]
        self.front_dist = msg.ranges[centerIndex]
	
        #print("Left Dist:", self.lwall_dist) 

    def control_loop(self, freq=40):
        r = rospy.Rate(freq)
        self.deltaT = 1 / freq
        # TODO: Implement PD control to follow left wall
        #   at desired distance of 1.5m, i.e. stay 1.5m 
        #       from left wall always
        
        rospy.Subscriber("scan", LaserScan, self.laser_cb)
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
            #print("error: ", self.err)
            #self.err_rate = (self.err - self.prev_err) / self.deltaT
            self.dq.append(self.err)
            self.dq.popleft()
            self.err_rate = sum(self.dq)/(self.deltaT*5) 
            #print("DQ: ", self.dq)
            
            
            #print("Error Rate: ", self.err_rate)
            # Update the desired angular velocity using PD
            self.steer = (self.kp*self.err + self.kd*self.err_rate)
            print("KP Impact: ", self.kp * self.err)
            print("KD Impact: ", self.kd * self.err_rate)
            if (self.steer < -2047):
            	self.steer = -2047
            if (self.steer > 2047):
            	self.steer = 2047
            #self.steer = 0
            #print("steer val ", self.steer)   
            # TODO: Set gas vel
            self.gasvel = self.des_vel.linear.x
            #print("gas vel:",self.gasvel)
            if (self.front_dist < 0.4):
                self.gasvel = 0
                 
                print("Too close")
            # TODO: publish steering and gas to topics
            self.accel_pub.publish(Float32(self.gasvel))
            self.steer_pub.publish(Float32(self.steer))

            # Move robot
            r.sleep()

    def clean_shutdown(self):
        # TODO: Publish zero velocities to all topics
        accel_pub = rospy.Publisher("/accelerator_cmd", Float32, queue_size=10)
        steer_pub = rospy.Publisher("/steering_cmd", Float32, queue_size=10)
        
        steer_pub.publish(Float32(0))
        accel_pub.publish(Float32(0))

if __name__=="__main__":
    rospy.init_node("wall_follow", anonymous=True)

    try:
        x = WallFollower()
        x.control_loop()
    except rospy.ROSInterruptException:
        x.clean_shutdown()
        print("shutdown")
        pass
