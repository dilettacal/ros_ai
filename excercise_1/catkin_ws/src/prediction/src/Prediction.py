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
        #Aufgabe 1.5
        #Publisher to /camera/input/specific/number
        self.prediction_publisher = rospy.Publisher("/camera/input/specific/number",
                                                        Int32,
                                                        queue_size=1)
        
        self.spec_pred = []
        self.session = k.get_session()
        self.graph = tf.get_default_graph()
        self.graph.finalize() #thread-safe
        rospy.loginfo("Model loaded!")
        
        #Test
        print("Model summary: ")
        print(self.cnn_model.summary())


    def callback_images(self, data):
        print("Header")
        rospy.loginfo(data.header.seq) 
        image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        """
        cv2.IMREAD_COLOR : Loads a color image. Any transparency of image will be neglected. It is the default flag.
        cv2.IMREAD_GRAYSCALE : Loads image in grayscale mode
        cv2.IMREAD_UNCHANGED : Loads image as such including alpha channel
        """
        image_expanded = np.expand_dims(image, axis=0) #(1,28,28)
        image_expanded_pred = np.expand_dims(image_expanded,axis=3)
        
        prediction_ohe =  self.cnn_model.predict(image_expanded_pred) #one hot encoded
        
        prediction = np.argmax(prediction_ohe, axis=None, out=None)
        print("Predicted number:", prediction)
        self.spec_pred.append(prediction)#Speichert die Prediction
        
        self.publish_prediction(prediction)
        
        
        
    def publish_prediction(self, pred):
        print("Published!")
        self.prediction_publisher.publish(pred)
        

    def callback_check(self, data):
        rospy.loginfo("Check " + rospy.get_caller_id())
        rospy.loginfo(data.header.seq)   

    def specific_subscriber(self):
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
        pred.specific_subscriber()
        
        
        
        #Aufgabe 1.6
        #Subscriber (Check) to /camera/output/specific/check

        while not rospy.is_shutdown():
          rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
