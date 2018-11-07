#!/usr/bin/env python
"""Excercise 2 """
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


def callback(data):
    rospy.loginfo("Random data received from " + rospy.get_caller_id())
    #rospy.loginfo(rospy.get_caller_id() + "Info %s received", data.data)

def random_listener():
    node_name = 'subscriber1'
    topic_name = '/camera/output/random/compressed_img_msgs'
    output_random_topic = '/camera/output/random/number'
    rospy.init_node(node_name, anonymous=True)
    #Subscriber1
    rospy.Subscriber(topic_name,CompressedImage, callback)


    #TODO: Check the data type,probably it is an int
    rospy.Subscriber(output_random_topic, CompressedImage, callback)
    rospy.spin()

if __name__ == '__main__':
    random_listener()
