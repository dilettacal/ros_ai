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

    def setup_model(self):
        #define model structure as in mnist_cnn_modified
        model = Sequential()
        model.add(Conv2D(filters=32,
                         kernel_size=(3, 3),
                         activation='relu',
                         input_shape=input_shape))
        model.add(Conv2D(filters=64,
                         kernel_size=(3, 3),
                         activation='relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(rate=0.25))
        model.add(Flatten())
        model.add(Dense(units=128,
                        activation='relu'))
        model.add(Dropout(rate=0.5))
        model.add(Dense(units=num_classes,
                        activation='softmax'))

        model.compile(loss=keras.losses.categorical_crossentropy,
                      optimizer=keras.optimizers.Adadelta(),
                      metrics=['accuracy'])
        return model



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
