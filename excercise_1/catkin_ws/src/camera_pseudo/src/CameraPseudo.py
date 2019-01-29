#!/usr/bin/env python

#Libraries
#OpenCV
from cv_bridge import CvBridge
import cv2

#numpy
import numpy as np
#ROS Libraries
import rospy
#Dataset
from keras.datasets import mnist
#ROS Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32

SPECIFIC_VALUE = 6  # value can be changed for different test-cases. This is not the labeled value but an index
PUBLISH_RATE = 0.30  # hz, standard == 3
USE_WEBCAM = False

class CameraPseudo:
    def __init__(self):
        self.cv_bridge = CvBridge() 

        self.publisher_webcam_comprs = rospy.Publisher("/camera/output/webcam/compressed_img_msgs",
                                                       CompressedImage,
                                                       queue_size=1)
        #Nicht relevant fuer Uebung
        if USE_WEBCAM:
            self.input_stream = cv2.VideoCapture(0)
            if not self.input_stream.isOpened():
                raise Exception('Camera stream did not open\n')

        self.publisher_specific_comprs = rospy.Publisher("/camera/output/specific/compressed_img_msgs",
                                                         CompressedImage,
                                                         queue_size=1)

        self.publisher_specific_check = rospy.Publisher("/camera/output/specific/check",
                                                        Bool,
                                                        queue_size=1)

        rospy.Subscriber('/camera/input/specific/number',
                         Int32,
                         self.camera_specific_callback)

        self.publisher_random_comprs = rospy.Publisher("/camera/output/random/compressed_img_msgs",
                                                       CompressedImage,
                                                       queue_size=1)

        self.publisher_random_number = rospy.Publisher("/camera/output/random/number",
                                                       Int32,
                                                       queue_size=1)

        # use mnist data as pseudo webcam images
        (_, _), (self.images, self.labels) = mnist.load_data()

        rospy.loginfo("Publishing data...")

    def camera_specific_callback(self, msg):
        # check if input is same as defined value
        result = True if msg.data == self.labels[SPECIFIC_VALUE] else False

        # publish result
        self.publisher_specific_check.publish(result)

    def publish_data(self, verbose=0):
        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            self.publish_specific(verbose)
            self.publish_random(verbose)

            # Note:
            # reactivate for webcam image. Pay attention to required subscriber buffer size.
            # See README.md for further information
            if USE_WEBCAM:
                self.publish_webcam(verbose)

            rate.sleep()

    def publish_specific(self, verbose=0):
        image = self.images[SPECIFIC_VALUE]

        # convert to msg
        compressed_imgmsg = self.cv_bridge.cv2_to_compressed_imgmsg(image)

        # publish data
        self.publisher_specific_comprs.publish(compressed_imgmsg)#Bild geschickt

        if verbose:
            rospy.loginfo("Begin publish_specific....")
            rospy.loginfo(compressed_imgmsg.header.seq)
            rospy.loginfo(compressed_imgmsg.format)
            rospy.loginfo("End publish_specific")

    def publish_random(self, verbose=0):
        # get random number
        rand_int = np.random.randint(0, len(self.labels), dtype='int')

        # get image and number based on random value
        image = self.images[rand_int]
        number = self.labels[rand_int]

        # convert to msg
        compressed_imgmsg = self.cv_bridge.cv2_to_compressed_imgmsg(image)

        # publish data
        self.publisher_random_comprs.publish(compressed_imgmsg)
        self.publisher_random_number.publish(number)

        if verbose:
            rospy.loginfo("Begin publish_random....")
            rospy.loginfo(compressed_imgmsg.header.seq)
            rospy.loginfo(compressed_imgmsg.format)
            rospy.loginfo("Label: " + str(number))
            rospy.loginfo("End publish_random.")

    def publish_webcam(self, verbose=0):
        if self.input_stream.isOpened():
            success, frame = self.input_stream.read()
            msg_frame = self.cv_bridge.cv2_to_compressed_imgmsg(frame)
            self.publisher_webcam_comprs.publish(msg_frame.header, msg_frame.format, msg_frame.data)

            if verbose:
                rospy.loginfo(msg_frame.header.seq)
                rospy.loginfo(msg_frame.format)


def main():
    rospy.loginfo("Publish rate: ")
    rospy.loginfo(PUBLISH_RATE)
    try:
        # register node
        rospy.init_node('camera_pseudo', anonymous=False)

        # init CameraPseudo
        cam = CameraPseudo()

        # start publishing data
        cam.publish_data(verbose=0)

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
