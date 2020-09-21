from PyQt5.QtGui import QPixmap
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import numpy as np

class CameraReader:
    def __init__(self):
        rospy.init_node('CarViewListener', anonymous=True)
        rospy.Subscriber("/master/camera_node/image/compressed", CompressedImage, self.publish_image, queue_size=1, buff_size=2**24)
        self.bridge = CvBridge()
        self.callbacks = []
        self.skip_frames = False

    def subscribe(self, callback):
        self.callbacks.append(callback)

    def publish_image(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        if not self.skip_frames:
            for cb in self.callbacks:
                cb(image_np, data)
    
    def skip(self, s):
        self.skip_frames = s
