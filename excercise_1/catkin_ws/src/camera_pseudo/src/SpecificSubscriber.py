#!/usr/bin/env python
"""Exercise 1 """
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


def callback_images(data):
    rospy.loginfo("Specific data received from " + rospy.get_caller_id())
    rospy.loginfo(data.header.seq)


def callback_check(data):
    rospy.loginfo("Check " + rospy.get_caller_id())
    rospy.loginfo(data.header.seq)


def specific_listener():
    rospy.loginfo("Start listener")
    node_name = 'img_subscriber1'
    specific_topic = '/camera/output/specific/compressed_img_msgs'
    specific_check_topic = '/camera/output/specific/check'
    rospy.init_node(node_name, anonymous=True)
    #Subscriber1
    rospy.Subscriber(specific_topic,CompressedImage, callback_images)
    rospy.Subscriber(specific_topic,Bool, callback_check)

    rospy.spin()


if __name__ == '__main__':
    specific_listener()
