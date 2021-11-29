#!/usr/bin/env python3
import rospy # Library for ROS-python
from geometry_msgs.msg import Twist # Library for expressing velocity in free space. 3 parametersfor linear and angular components
from turtlesim.msg import Pose # Library to get the pose of the turtle bot
import math
import time
from std_srvs.srv import Empty
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

x = 0
y = 0
z = 0
yaw = 0

def poseCallback(pose_message):
    # pose_message -> message from turtlebot for its pose
    global x
    global y,z,yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

def move(speed, distance, is_forward):
    velocity_message = Twist() # Twist is the message type for velocity

    # Current Location of the turtlebot
    global x,y
    x0 = x
    y0 = y

    # To check whether to go forward or backward depending upon the input
    if(is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # To keep the loop at 10hz

    cmd_vel_topic = '/turtle1/cmd_vel' #Name of the Topic of interest
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    #queue_size = size of the outgoing message queue used for asynchronous publishing

    while True:
        rospy.loginfo("Turtle moves forward")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep() # can throw a rospy.ROSInterruptException if the sleep is interrupted by shutdown.

        distance_moved = distance_moved + abs(0.5*math.sqrt(((x-x0)**2 ) + ((y - y0) ** 2)))
        print(distance_moved)

        if not(distance_moved < distance):
            rospy.loginfo("reached")
            break

    # Stop the turtlebot
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise):

    global yaw
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    #get current location
    theta0 = yaw
    angular_speed = math.radians(abs(angular_speed_degree))

    if(clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = 'turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

    t0 = rospy.Time.now().to_sec() # Get the current time in seconds

    while True:
       rospy.loginfo("Turtle rotates")
       velocity_publisher.publish(velocity_message)
       t1 = rospy.Time.now().to_sec()
       current_angle_degree = (t1-t0)*angular_speed_degree
       loop_rate.sleep()

       if not(current_angle_degree > relative_angle_degree):
           rospy.loginfo("reached")
           break
    
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message) 

def go_to_goal(x_goal, y_goal):
    global x
    global y, z, yaw

    velocity_message = Twist()
    cmd_vel_topic = 'turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
    dist = []
    ang = []
    integral = 0
    derivative = 0
    err_angle = 0
    integral_a = 0
    derivative_a = 0
    integral_prior_a = 0
    integral_prior_l = 0
    previous_distance = 0
    pre_err_angle = 0

    while True:
        Kp_linear = 0.5
        Ki_linear = 0 #0.01
        Kd_linear = 0 #0.1
        dt = 0.01

        distance = abs(0.5*math.sqrt(((x_goal-x)**2 ) + ((y_goal - y) ** 2)))
        integral = integral_prior_l + distance*dt
        derivative = (distance - previous_distance)/dt
        dist.append(distance)
        


        linear_speed = Kp_linear * distance + Ki_linear * integral + Kd_linear * derivative

        Kp_angular = 4.0
        Ki_angular =  0#0.0001
        Kd_angular = 0 #0.1


        desired_angle_goal = math.atan2(y_goal - y,x_goal - x)
        err_angle = desired_angle_goal - yaw
        integral_a = integral_prior_a + err_angle*dt
        derivative_a = (err_angle - pre_err_angle)/dt
        angular_speed =  err_angle * Kp_angular + Ki_angular*integral_a + Kd_angular * derivative_a


        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        ang.append(err_angle)

        velocity_publisher.publish(velocity_message)
        previous_distance = distance
        pre_err_angle = yaw
        integral_prior_l = integral
        integral_prior_a = integral_a


        print("x = ", x,"y = ",y)

        if(distance < 0.01):
            break
    
    df = pd.DataFrame(dist)
    df2 = pd.DataFrame(ang) 
    
    # saving the dataframe 
    df.to_csv('distance12.csv')  
    df2.to_csv("angle12.csv") 
    # siz = len(dist)
    # si = np.linspace(0,1,siz)

    # # plt.axhline(y = desired_dist, color = 'r', linestyle = '-')
    # size = len(dist)
    # plt.plot(dist,si)
    # plt.xlabel('time')
    # # naming the y axis
    # plt.ylabel('Distance travelled')
 
    # # giving a title to my graph
    # plt.title('Distance travelled vs time')

def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw

    if relative_angle_radians < 0:
        clockwise = 1

    else: 
        clockwise = 0
    
    print(relative_angle_radians)
    print(desired_angle_radians)
    rotate(30, math.degrees(abs(relative_angle_radians)),clockwise)


if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose',anonymous= True)
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

        position_topic = 'turtle1/pose'
        pose_subscriber = rospy.Subscriber(position_topic,Pose,poseCallback)
        time.sleep(2)
        #setDesiredOrientation(90)

        go_to_goal(1.8,1.0)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")