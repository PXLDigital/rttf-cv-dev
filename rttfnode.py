import os
from stat import ST_CTIME, ST_MODE, S_ISREG

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time
from edgecar_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage

from scanner import Scanner

import config as cfg
import masks as masks

class DrivingModule:
    def __init__(self):
        self.vehicle_name = rospy.get_param("~veh", "edgecar")
        self.start_driving_subscription_name = "/{}/ai/start_driving".format(self.vehicle_name)
        self.image_subscription_name = '/{}/camera_node/image/compressed'.format(self.vehicle_name)
        self.pub_car_cmd_name = '/{}/wheels_driver_node/wheels_cmd'.format(self.vehicle_name)

        self.sub_start_using_ai = rospy.Subscriber(self.start_driving_subscription_name, Bool, self.start_using_ai)

        self.pub_car_cmd = rospy.Publisher(self.pub_car_cmd_name, WheelsCmdStamped, queue_size=1)
        self.wheels_cmd = WheelsCmdStamped()

        self.scanner = Scanner()

        self.image_subscriber = None  # will be used later on when we start using AI
        self.running = False
        self.kl = None

        self.counter = 0

        self.bridge = CvBridge()

def start_using_ai(self, data):        
        if data.data:
            rospy.loginfo("[{}] Start Driving making use of CV AI : {}".format(rospy.get_name(), data.data))
            self.running = True
            self.image_subscriber = rospy.Subscriber(self.image_subscription_name, CompressedImage, self.process_img)
            rospy.loginfo("[{}] Subscribed to image topic".format(rospy.get_name()))
        elif not data.data:
            self.stop_running()

def process_img(self, data):
    if data:
        try:
            angle = cfg.MAX_ANGLE
            throttle = cfg.MAX_THROTTLE

            img = self.bridge.compressed_imgmsg_to_cv2(data)
            hsv_data = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            mask = None
            image_result = None

            mask_bounds = masks.limits[cfg.LOCATION]
            mask = cv2.inRange(hsv_data, np.array(mask_bounds[0][0]), np.array(mask_bounds[0][1]))
            mask2 = cv2.inRange(hsv_data, np.array(mask_bounds[1][0]), np.array(mask_bounds[1][1]))
            total_mask = mask | mask2
            total_mask = cv2.erode(total_mask, (3, 3), 2)
            total_mask = cv2.dilate(total_mask, (3, 3), 2)
            
            image_result = cv2.bitwise_and(img, img, mask=total_mask)
            
            self.scanner.set_image(total_mask)
            #d = self.scanner.find_direction()
            #p = Plotter.plot_direction(image_result, d)
            (target, absdir, left, right) = self.scanner.direction_exp()
            #p = Plotter.plot_direction_exp(image_result, d)

            angle = angle * absdir
            throttle = throttle

            self.wheels_cmd.header.stamp = data.header.stamp
            self.wheels_cmd.velocity = throttle
            self.wheels_cmd.rotation = angle
            self.pub_car_cmd.publish(self.wheels_cmd)
        
            if self.counter % 20 == 0:
                rospy.loginfo("[{}] outputs : {}".format(rospy.get_name(), outputs))
                self.counter = 0
            self.counter += 1

        except CvBridgeError as e:
            rospy.logerr("error cvbridge stuff : ", e)
    
    def stop_running(self):
        rospy.loginfo("[{} Must stop running AI container".format(rospy.get_name()))
        if not self.image_subscriber is None:
            self.image_subscriber.unregister()
            self.image_subscriber = None
            rospy.logdebug("[{}] Finished unsubscribing from image stream".format(rospy.get_name()))
        self.running = False

    def on_shutdown(self):
        rospy.loginfo('[DrivingModule] onShutdown ...')


if __name__ == '__main__':
    rospy.init_node('driving_node', anonymous=False)
    driving_module = DrivingModule()
    rospy.on_shutdown(driving_module.on_shutdown)
    rospy.spin()