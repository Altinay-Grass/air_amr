#!/usr/bin/env python

import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseListenerNode:
    def __init__(self):
        rospy.init_node('pose_listener_node', anonymous=True)
        self.file_prefix = rospy.get_param('~file_prefix', 'data')  # Prefix for the YAML file
        self.save_period = rospy.get_param('~save_period', 10)  # Save period in seconds
        self.topic_name = rospy.get_param('~topic_name', 'amcl_pose')  # Replace 'your_topic' with the actual topic name

        self.subscriber = rospy.Subscriber(self.topic_name, PoseWithCovarianceStamped, self.callback)
        self.yaml_data = {}  # Dictionary to store the received data

    def callback(self, msg):
        # Extract the pose data from the message
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        # Store the pose data in the yaml_data dictionary
        self.yaml_data['position'] = {'x': position.x, 'y': position.y, 'z': position.z}
        self.yaml_data['orientation'] = {'x': orientation.x, 'y': orientation.y, 'z': orientation.z, 'w': orientation.w}

    def save_data(self):
        # Save the yaml_data dictionary as a YAML file
#        filename = self.file_prefix + str(rospy.get_time()) + '.yaml'
        if self.yaml_data:
            filename = self.file_prefix + '.yaml'
            with open(filename, 'w') as yaml_file:
                yaml.dump(self.yaml_data, yaml_file)
            rospy.loginfo('Data saved to file: %s', filename)
            self.yaml_data = {}  # Clear the dictionary after saving data
        else:
            rospy.loginfo('No data to save.')

    def run(self):
        rate = rospy.Rate(1.0 / self.save_period)  # Rate at which to save data
        while not rospy.is_shutdown():
            self.save_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PoseListenerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass