#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

min_distance = 0.3
current_position = {'x': None, 'y': None, 'z':None}
attractive_force_gain = 0.3
repulsive_force_gain = 0.5
goal_x = -7.951355885839356e-05
goal_y = 1.5351194885603948e-06
    

# right, front, left, shortest distance direction 
# 1 front is shortest, 2 right is shortest, 3 left is shortest
lookup_table = {
    (0, 0, 0,1): (0.06, 0),      # No obstacles in any region, go forward
    (0, 0, 0,2): (0, 0.1),      # No obstacles in any region, go right
    (0, 0, 0,3): (0, -0.1),      # No obstacles in any region, go left

    (1, 0, 0,1): (0.06, 0),     # Obstacle on the right, go forward
    (1, 0, 0,3): (0, -0.1),     # Obstacle on the right, turn left

    (0, 1, 0,2): (0, -0.06),     # Obstacle in front, go left
    (0, 1, 0,3): (0, 0.06),     # Obstacle in front, go right
    #(0, 1, 0,0): (0, -0.2),     # Obstacle in front, go backward

    (0, 0, 1,2): (0, 0.1),      # Obstacle on the left, turn right
    (0, 0, 1,1): (0.06, 0),      # Obstacle on the left, go foward

    (1, 1, 0,0): (0, -0.1),     # Obstacle on the right and front, turn left
    (0, 1, 1,0): (0, 0.1),      # Obstacle in front and left, turn right
    (1, 0, 1,0): (0.06, 0),      # Obstacle on the right and left, go forward
    (1, 1, 1,0): (0, -0.06)      # Obstacle on all sides, go backward 
}


def avoid_obstcle():
    global pub 
    rospy.init_node('turtlebot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, divide_zone_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
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
    
def odom_callback(data):
    global current_position
    current_position['x'] = data.pose.pose.position.x
    current_position['y'] = data.pose.pose.position.y
    current_position['z'] = data.pose.pose.orientation.z


def distance_to_goal():
    # calculate the distance btwn the current robot pos. and the goal
    global  goal_y, goal_x
    
    distance = ((goal_x - current_position['x']) ** 2 + (goal_y - current_position['y']) ** 2) ** 0.5
    return distance

def calculate_angle_to_goal():
    global goal_x, goal_y, current_position
    theta = current_position['z']
    # Calculate the angle between the current orientation (theta) of the robot and the direction towards the goal position
    angle_to_goal = math.atan2(goal_y - current_position['y'], goal_x - current_position['x'])
    
    # Normalize the angle to be within the range [-pi, pi]
    while angle_to_goal - theta > math.pi:
        angle_to_goal -= 2 * math.pi
    while angle_to_goal - theta < -math.pi:
        angle_to_goal += 2 * math.pi
    
    return angle_to_goal

def calculate_repulsive_force(regions, repulsive_force_gain):
    repulsive_force = 0
    for region in regions.values():
        # If the distance is too small, set the force to a large value to avoid getting too close to obstacles
        repulsive_force += repulsive_force_gain / max(region, 0.001)

    return repulsive_force
    
def navigation(regions):
    twist_msg = Twist()
    
    # Calculate attractive force towards the goal
    goal_distance =  distance_to_goal()  
    angle_to_goal = calculate_angle_to_goal()        
    attractive_force_linear = attractive_force_gain * goal_distance
    attractive_force_angular = attractive_force_gain * angle_to_goal

    # Calculate repulsive force from obstacles
    repulsive_force_linear = calculate_repulsive_force(regions,repulsive_force_gain)

    # Combine the attractive and repulsive forces
    linear_velocity = attractive_force_linear - repulsive_force_linear
    angular_velocity = attractive_force_angular

    # Find the distance to the goal from all the open regions
    front_distance = goal_distance if regions['front'] < min_distance else float('inf')
    right_distance = goal_distance if regions['right'] < min_distance else float('inf')
    left_distance = goal_distance if regions['left'] < min_distance else float('inf')

    #define the lookup table keys
    key = (
        1 if regions['right'] < min_distance else 0,
        1 if regions['front'] < min_distance else 0,
        1 if regions['left'] < min_distance else 0,
        1 if front_distance < right_distance and front_distance < left_distance else 
        2 if right_distance < left_distance and right_distance < front_distance else 
        3 if left_distance < right_distance and left_distance < front_distance else 0
    )
    
    #chose the right direction of the robot based on the look up table
    linear_velocity, angular_velocity = lookup_table.get(key, (linear_velocity, angular_velocity))

    # Publish
    twist_msg.linear.x = linear_velocity
    twist_msg.angular.z = angular_velocity
    pub.publish(twist_msg)
    




if __name__ == '__main__':
    try:
        avoid_obstcle()
    except rospy.ROSInterruptException:
        pass