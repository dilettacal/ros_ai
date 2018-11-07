#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import tensorflow as tf
from keras import backend as k
from cv_bridge import CvBridge
from keras.models import load_model
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32
#from catkin_ws.src.camera_pseudo.src import CameraPseudo

class Prediction:
    def __init__(self):
        self.cv_bridge = CvBridge()
        #Model used: The given model
        self.model = load_model("/home/ros18/KI/ros_robotics/excercise_1/ai_train/models/weights-best.hdf5")
        self.session = k.get_session()
        self.graph = tf.get_default_graph()
        self.graph.finalize()
        #Test
        print(self.model.summary())


def main():
    try:
        # register node
        rospy.init_node('prediction', anonymous=False)


        #Publish on
        #//camera/input/specific/number #CameraPseudo is subscriber
        rospy.Publisher("/camera/input/specific/number ", Int32,queue_size=1)

        #Verify on
        #/camera/output/specific/check

        # init CameraPseudo
        pred = Prediction()
        #camera = CameraPseudo()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
