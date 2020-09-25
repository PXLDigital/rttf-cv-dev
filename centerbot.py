import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors

from plotter import Plotter
from scanner import Scanner

from PyQt5.QtWidgets import QApplication
from mainwindow import MainWindow
from camera_reader import CameraReader
import rospy
import sys, signal 


def on_close(sig, frame):
    print("Shutting down")
    #stop_message = Joy()
    #stop_message.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    sys.exit()

if __name__ == "__main__":
    app = QApplication([])

    win = MainWindow("RTTF Trainer")
    win.show()

    signal.signal(signal.SIGINT, on_close)

    app.exec_()
    rospy.spin


'''
# Load an image
img = cv2.imread('test/straight.png', cv2.COLOR_BGR2RGB)
hsv_data = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

Plotter.plot_hsv(img)
#Plotter.plot_img(img)

lower = (110, 250, 250)
upper = (120, 255, 255)

#Plotter.plot_colors_hsv(lower, upper)

scanner = Scanner()
mask = None
while mask == None:    
    
    mask = cv2.inRange(hsv_data, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
    scanner.set_image(mask)
    d = scanner.find_direction()
    Plotter.plot_direction(result, d)
'''

'''
def image_callback(self, data):
        if data:
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                cv_image = cv2.resize(cv_image, (160, 120))
                outputs = self.kl.run(cv_image)

                self.wheels_cmd.header.stamp = data.header.stamp
                self.wheels_cmd.velocity = outputs[0]
                self.wheels_cmd.rotation = outputs[1]
                self.pub_car_cmd.publish(self.wheels_cmd)

                if self.counter % 20 == 0:
                    rospy.loginfo("[{}] outputs : {}".format(rospy.get_name(), outputs))
                    self.counter = 0
                self.counter += 1
            except CvBridgeError as e:
                rospy.logerr("error cvbridge stuff : ", e)
'''