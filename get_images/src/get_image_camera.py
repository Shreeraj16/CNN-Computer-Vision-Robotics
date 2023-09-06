#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import math
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tensorflow.keras.models import load_model


class ImageFeeder():
    def __init__(self):
        # camera objects
        self.bridge_object = CvBridge()
        self.cam_topic = '/camera/rgb/image_raw'
        self.img_sub = rospy.Subscriber(
            self.cam_topic, Image, self.cameraCallback)
        # model objects
        self.model = load_model(os.path.join(
            os.path.dirname("/home/user/catkin_ws/src/dlrepo/traffic_sign_data_set/"), "cnn_traffic_sign.h5"))
        print('LOADED MODEL')

    def cameraCallback(self, cam_msg):
        # can return header, height, width, encoding (rgb8), step, data
        try:
            img = self.bridge_object.imgmsg_to_cv2(
                cam_msg, desired_encoding='bgr8')
            #print('LOADED IMAGE')
        except CvBridgeError as cve:
            print('Failing to load image')
            rospy.loginfo(cve)
        h, w, ch = img.shape
        # cropping the image to only see the important stuff
        cropped_img = img[500:1020, 900:1120]

        cv2.imshow("RES", cropped_img)
        cv2.waitKey(1)

        self.predict(cropped_img)

    def predict(self, img_to_pred):

        img = cv2.resize(img_to_pred, dsize=(32, 32))
        img = img.reshape(1, 32, 32, 3)
        y_pred = self.model.predict_classes(img)
        if y_pred:
            print('Predicted class: '+str(y_pred))
        else:
            print('-------------------------')

    def clean_up(self):
        cv2.destroyAllWindows()


if __name__ == '__main__':
    imagefeeder = ImageFeeder()
    rospy.init_node('image_feed__node', anonymous=True)

    rate = rospy.Rate(1)
    ctrl_c = False

    def shutdownhook():
        imagefeeder.clean_up()
        rospy.loginfo('shutdown time!')
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()
