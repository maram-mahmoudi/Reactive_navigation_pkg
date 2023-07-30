#!/usr/bin/env python3

import rospy
import random
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist

class PersonObstaclePlugin:
    def __init__(self):
        rospy.init_node("person_obstacle_plugin")
        self.model_state_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        
        self.model_name = "person_walking"

    def model_states_callback(self, data):
        try:
            # Find the index of the person model in the model states message
            person_idx = data.name.index(self.model_name)

            # Get the current pose of the person model
            person_pose = data.pose[person_idx]

            # Set the desired position of the person model (modify these values)
            desired_x = 29.461601
            desired_y = 8.700830 
            desired_z = -0.000001
     
            # Update the position of the person model in the data received
            person_pose.position.x = desired_x
            person_pose.position.y = desired_y
            person_pose.position.z = desired_z

        except ValueError:
        # The person model was not found in the received data
            pass


    def update(self):
        # Generate random linear and angular velocities 
        linear_vel_x = random.uniform(0.5, 2.0) + random.uniform(-0.1, 0.1)
        angular_vel_z = random.uniform(-1.0, 1.0)

        
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel_x
        twist_msg.angular.z = angular_vel_z

        # Create and set the ModelState message
        model_state_msg = ModelState()
        model_state_msg.model_name = self.model_name
        model_state_msg.twist = twist_msg

        self.model_state_publisher.publish(model_state_msg)

if __name__ == "__main__":
    try:
        person_obstacle = PersonObstaclePlugin()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            person_obstacle.update()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

