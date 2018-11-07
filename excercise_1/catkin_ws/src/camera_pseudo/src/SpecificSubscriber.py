#!/usr/bin/env python
"""Exercise 1 """
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


def callback_images(data):
    rospy.loginfo("Specific data received from " + rospy.get_caller_id())
    #rospy.loginfo(rospy.get_caller_id() + "Info %s received", data.data)

def callback_check(data):
    rospy.loginfo("Check " + rospy.get_caller_id())

def specific_listener():
    node_name = 'img_subscriber1'
    specific_topic = '/camera/output/specific/compressed_img_msgs'
    rospy.init_node(node_name, anonymous=True)
    #Subscriber1
    rospy.Subscriber(specific_topic,CompressedImage, callback_images)

    rospy.spin()

def specific_check():
    node_name = 'check_subscriber1'
    specific_check_topic = '/camera/output/specific/check'
    rospy.init_node(node_name, anonymous=True)
    #Subscriber1
    rospy.Subscriber(specific_topic,Bool, callback_check)

    rospy.spin()

if __name__ == '__main__':
    specific_listener()
    specific_check()
