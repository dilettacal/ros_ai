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
#catkin_ws/src/camera_pseudo/src/CameraPseudo.py
import CameraPseudo

class Prediction:
    def __init__(self):
        self.cv_bridge = CvBridge()
        #Model used: The given model
        self.model = load_model("/home/ros18/KI/ros_robotics/excercise_1/ai_train/models/weights-best.hdf5")
        self.session = k.get_session()
        self.graph = tf.get_default_graph()
        self.graph.finalize() #thread-safe
        rospy.loginfo("Model loaded!")
        #Test
        #print(self.model.summary())

    def callback(self, image_msg):
        target_size = (28, 28)
        #First convert the image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        cv_image = cv2.resize(cv_image, target_size)  # resize image
        np_image = np.asarray(cv_image)               # read as np array
        np_image = np.expand_dims(np_image, axis=0)   # Add another dimension for tensorflow
        np_image = np_image.astype(float)  # preprocess needs float64 and img is uint8
        np_image = preprocess_input(np_image)         # Normalize the data

        global graph                                  # This is a workaround for asynchronous execution
        with graph.as_default():
           preds = self.model.predict(np_image)            # Classify the image
           # decode returns a list  of tuples [(class,description,probability),(class, descrip ...
           pred_string = decode_predictions(preds, top=1)[0]   # Decode top 1 predictions
           msg_string.data = pred_string[0][1]
           msg_float.data = float(pred_string[0][2])
           pub.publish(msg_string)
           rospy.loginfo("Float: " + str(msg_float.data))
           #pub1.publish(msg_float)

def main():
    try:
        # register node
        rospy.init_node('prediction', anonymous=False)

        # init CameraPseudo
        camera = CameraPseudo()
        pred = Prediction()

        #Publish on
        #//camera/input/specific/number #CameraPseudo is subscriber
        #String oder Float32?
        pub = rospy.Publisher("/camera/input/specific/number ", Float32 ,queue_size=1)
        bridge = CvBridge()

        msg_string = String()
        msg_float = Float32()

        rospy.Subscriber("/camera/output/specific/check", Image, pred.callback, queue_size = 1, buff_size = 16777216)

        while not rospy.is_shutdown():
          rospy.spin()

        #Verify on
        #/camera/output/specific/check



        #rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
