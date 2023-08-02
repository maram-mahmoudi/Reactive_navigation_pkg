#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

min_distance = 0.5

lookup_table = {
    (0, 0, 0): (0.2, 0),      # No obstacles in any region, go forward
    (1, 0, 0): (0, -0.1),     # Obstacle on the right, turn left
    (0, 1, 0): (0, -0.2),        # Obstacle in front, go backward
    (0, 0, 1): (0, 0.1),      # Obstacle on the left, turn right
    (1, 1, 0): (0, -0.1),     # Obstacle on the right and front, turn left
    (0, 1, 1): (0, 0.1),      # Obstacle in front and left, turn right
    (1, 0, 1): (0, 0),        # Obstacle on the right and left, stop turning
    (1, 1, 1): (0, -0.2)         # Obstacle on all sides, go backward 
}

def distance_to_goal():
    goal_x = 1
    goal_y = 0
    current_x_pos =0 # this will probalby be received from pose or odom
    current_y_pos =0 # this will probalby be received from pose or odom

    distance = ((goal_x - current_y_pos) ** 2 + (goal_y - current_y_pos) ** 2) ** 0.5
    return distance

def calculate_angle_to_goal():
    goal_y, y, goal_x, x, theta=0
    # Calculate the angle between the current orientation (theta) of the robot and the direction towards the goal position
    angle_to_goal = math.atan2(goal_y - y, goal_x - x)
    
    # Normalize the angle to be within the range [-pi, pi]
    while angle_to_goal - theta > math.pi:
        angle_to_goal -= 2 * math.pi
    while angle_to_goal - theta < -math.pi:
        angle_to_goal += 2 * math.pi
    
    return angle_to_goal

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
    
    
    #min_distance = 0.4
   # linear_velocity = 0.3 
    # angular_velocity = 0.1
    # twist_msg = Twist()
    # 
    # if regions['front'] > min_distance and regions['left'] > min_distance and regions['right'] > min_distance:
       # #no obstacle  // can go right, left, or forward
      # # chose forward
        # twist_msg.linear.x = -linear_velocity
        # twist_msg.angular.z = 0
        # print("going forward, there s no obstacle")
    # elif regions['front'] > min_distance and regions['left'] > min_distance and regions['right'] < min_distance:
        ##right side is not safe // can go left or backward
        ##chose left 
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = -angular_velocity
        # print("going left")
    # elif regions['front'] > min_distance and regions['left'] < min_distance and regions['right'] > min_distance:
        ##left is not safe // can go right or forward
        ##chose right 
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = angular_velocity
        # print("going right")
    # elif regions['front'] < min_distance and regions['left'] > min_distance and regions['right'] > min_distance:
       ## front is not safe  // can go backward, right or left
        ##chose right
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = angular_velocity
        # print("going right")
    # elif regions['front'] < min_distance and regions['left'] < min_distance and regions['right'] < min_distance:
       ## nothing is safe // can go backward only
        # twist_msg.linear.x = linear_velocity
        # twist_msg.angular.z = 0
        # print("going backward")
    # elif regions['front'] < min_distance and regions['left'] < min_distance and regions['right'] > min_distance:
        ##only right is safe
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = angular_velocity
        # print("going right")
    # elif regions['front'] < min_distance and regions['left'] > min_distance and regions['right'] < min_distance:
        ##only left is safe
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = -angular_velocity
        # print("going left")
    # elif regions['front'] > min_distance and regions['left'] < min_distance and regions['right'] < min_distance:
        ##only front is safe
        # twist_msg.linear.x = -linear_velocity
        # twist_msg.angular.z = 0
        # print("going forward, my only choice")
    # else: 
        # print("i am stuck")
    #   
    # pub.publish(twist_msg)
    # 
     # Check if there is an obstacle in each region based on the minimum distance threshold
    obstacle_front = regions['front'] <= min_distance
    obstacle_right = regions['right'] <= min_distance
    obstacle_left = regions['left'] <= min_distance

    # Use the obstacle information to determine the action based on the lookup table
    linear_velocity, angular_velocity = lookup_table.get((obstacle_right, obstacle_front, obstacle_left), (0, 0))

    # Now, you have the linear and angular velocity values to control the robot
    # Publish the velocities to make the robot move accordingly
    twist_msg = Twist()
    twist_msg.linear.x = linear_velocity
    twist_msg.angular.z = angular_velocity
    pub.publish(twist_msg)


if __name__ == '__main__':
    try:
        avoid_obstcle()
    except rospy.ROSInterruptException:
        pass