#!/usr/bin/env python3
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
import numpy as np

data1 = 0 # Declaring a global variable

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.real_pose = rospy.Publisher('/rt_real_pose', Pose, queue_size = 10)
        self.noisy_pose = rospy.Publisher('/rt_noisy_pose', Pose, queue_size = 10) 
        self.rate = rospy.Rate(1) 

    #Callback function implementing the pose value received
    def callback(self, data):
        global data1
        data1 = data
        self.pose = data1
        self.pose.theta = round(self.pose.theta, 4)
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        
    def talker(self,timer):
        global data1
        self.real_pose.publish(data1)

    def talkernoisy(self,timer):
        global data1
        noise = data1
        # Adding gaussian noise with standard deviation of 10 and mean 0 to the pose
        noise.x = noise.x + np.random.normal(0,10) 
        noise.y = noise.y + np.random.normal(0,10)
        noise.theta = noise.theta + np.random.normal(0,10)
        noise.linear_velocity = noise.linear_velocity + np.random.normal(0,10)
        noise.angular_velocity = noise.angular_velocity + np.random.normal(0,10)
        self.noisy_pose.publish(noise) 

    def make_circle(self,timer):
        r = float(input("Radius =  "))
        w = float(input("Angular velocity(in rad/sec) = "))
        n = float(input("Number of turns =  "))
        
        vel_msg = Twist()
        
        t0 = rospy.Time.now().to_sec()        
        distance = 0
        angle = self.pose.theta
       
        while (2*3.1416*r*n) >= distance: 

            vel_msg.linear.x = w*r
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
    
         #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = w

            t1=rospy.Time.now().to_sec()
        
            distance = vel_msg.linear.x * (t1-t0)
            
            print('No. of turns = '+ str( distance/(2*3.1416*r)))
           
            self.velocity_publisher.publish(vel_msg)
            
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)
       
if __name__ == '__main__':
    try:
        
        x = turtlebot()
        rospy.Timer(rospy.Duration(5), x.talker)
        rospy.Timer(rospy.Duration(1), x.make_circle)
        rospy.Timer(rospy.Duration(5),x.talkernoisy)
        
        rospy.spin()
        
    except rospy.ROSInterruptException: pass
