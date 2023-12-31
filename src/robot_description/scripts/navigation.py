#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def avoid_obstcle():
    global pub 
    rospy.init_node('turtlebot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, divide_zone_callback)
    rate = rospy.Rate(1)  # 1hz

    while not rospy.is_shutdown():
        rate.sleep()
    
def divide_zone_callback(data):
    #divide the scan data into zones with a threshhold of 10 
    regions = {
    'right':       min(min(data.ranges[0:120]), 10),
    'front':       min(min(data.ranges[120:240]), 10),
    'left':        min(min(data.ranges[240:360]), 10),
    
    }

    navigation(regions)
    
def navigation(regions):
   
    min_distance = 0.4
    linear_velocity = 0.3 
    angular_velocity = 0.1
    twist_msg = Twist()
    
    if regions['front'] > min_distance and regions['left'] > min_distance and regions['right'] > min_distance:
       #no obstacle  // can go right, left, or forward
      # chose forward
        twist_msg.linear.x = -linear_velocity
        twist_msg.angular.z = 0
        print("going forward, there s no obstacle")
    elif regions['front'] > min_distance and regions['left'] > min_distance and regions['right'] < min_distance:
        #right side is not safe // can go left or backward
        #chose left 
        twist_msg.linear.x = 0
        twist_msg.angular.z = -angular_velocity
        print("going left")
    elif regions['front'] > min_distance and regions['left'] < min_distance and regions['right'] > min_distance:
        #left is not safe // can go right or forward
        #chose right 
        twist_msg.linear.x = 0
        twist_msg.angular.z = angular_velocity
        print("going right")
    elif regions['front'] < min_distance and regions['left'] > min_distance and regions['right'] > min_distance:
       # front is not safe  // can go backward, right or left
        #chose right
        twist_msg.linear.x = 0
        twist_msg.angular.z = angular_velocity
        print("going right")
    elif regions['front'] < min_distance and regions['left'] < min_distance and regions['right'] < min_distance:
       # nothing is safe // can go backward only
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = 0
        print("going backward")
    elif regions['front'] < min_distance and regions['left'] < min_distance and regions['right'] > min_distance:
        #only right is safe
        twist_msg.linear.x = 0
        twist_msg.angular.z = angular_velocity
        print("going right")
    elif regions['front'] < min_distance and regions['left'] > min_distance and regions['right'] < min_distance:
        #only left is safe
        twist_msg.linear.x = 0
        twist_msg.angular.z = -angular_velocity
        print("going left")
    elif regions['front'] > min_distance and regions['left'] < min_distance and regions['right'] < min_distance:
        #only front is safe
        twist_msg.linear.x = -linear_velocity
        twist_msg.angular.z = 0
        print("going forward, my only choice")
    else: 
        print("i am stuck")
      
    pub.publish(twist_msg)
    
    # 

if __name__ == '__main__':
    try:
        avoid_obstcle()
    except rospy.ROSInterruptException:
        pass