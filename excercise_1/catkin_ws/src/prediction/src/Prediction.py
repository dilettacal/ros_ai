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
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from keras import backend as K
#catkin_ws/src/camera_pseudo/src/CameraPseudo.py

class Prediction:
    def __init__(self):
        self.cv_bridge = CvBridge()
        #Aufgabe 1.4 - Include the model in Prediction.py
        self.cnn_model = load_model("/home/ros18/KI/ros_robotics/excercise_1/ai_train/models/weights-best.hdf5")
        self.cnn_model._make_predict_function() #Initialize before threading
        self.spec_sequences = []
        self.session = k.get_session()
        self.graph = tf.get_default_graph()
        self.graph.finalize() #thread-safe
        rospy.loginfo("Model loaded!")
        
        #Test
        print("Model summary: ")
        print(self.cnn_model.summary())


    def callback_images(self, data):
        rospy.loginfo('received image of type: "%s"' %data.format) #jpg
        rospy.loginfo(data.header.seq)
        img_np_arr = np.fromstring(data.data, np.uint8)
        #rospy.loginfo(np_arr)
        #https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_gui/py_image_display/py_image_display.html
        encoded_img = cv2.imdecode(img_np_arr, cv2.IMREAD_GRAYSCALE)
        print(encoded_img.shape)
        #K.expand_dims(x, 1)
        #ValueError: Error when checking input: expected conv2d_1_input 
        #to have 4 dimensions, but got array with shape (1, 28, 28)
        
        #ValueError: Error when checking input: expected conv2d_1_input to have shape (28, 28, 1) but got array with shape (1, 28, 28)
        expanded_img = np.expand_dims(encoded_img,axis=0)
        print(expanded_img.shape)
        prediction =  self.cnn_model.predict(expanded_img)
        print(prediction)
        
        self.spec_sequences.append((data.header.seq, prediction))
        print(self.spec_sequences)
       # rospy.loginfo(image_np)
       # cv2.imshow('cv_img', image_np)
        #return data

    def callback_check(self, data):
        rospy.loginfo("Check " + rospy.get_caller_id())
        rospy.loginfo(data.header.seq)   

    def specific_listener(self):
        specific_topic = '/camera/output/specific/compressed_img_msgs'
        rospy.Subscriber(specific_topic,
                         CompressedImage, 
                         self.callback_images, 
                         queue_size = 1)
       


def main():
    try:
        # register node
        rospy.init_node('prediction', anonymous=False)

        # init CameraPseudo
        pred = Prediction()
        
        #Aufgabe 1.2
        #Subscriber to /camera/output/specific/compressed_img_msgs
        pred.specific_listener()
        
        #Aufgabe 1.5
        #Publisher to /camera/input/specific/number
        
        
        #Aufgabe 1.6
        #Subscriber (Check) to /camera/output/specific/check

        while not rospy.is_shutdown():
          rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
