from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QImage, QPainter

from cv_bridge import CvBridge, CvBridgeError

from plotter import Plotter
from scanner import Scanner

import cv2
import numpy as np

class FilterView(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)
        self.image = QImage()
        self.bridge = CvBridge()
        self.scanner = Scanner()

    def image_data_slot(self, image_data, raw):
        img = self.bridge.compressed_imgmsg_to_cv2(raw)
        #img = cv2.imread(, cv2.COLOR_BGR2RGB)
        hsv_data = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #scanner = Scanner()
        mask = None
        image_result = None
        mask = cv2.inRange(hsv_data, np.array([0,100,50]), np.array([10,255,255]))
        mask2 = cv2.inRange(hsv_data, np.array([170,100,50]), np.array([180,255,255]))
        total_mask = mask | mask2
        total_mask = cv2.erode(total_mask, (3, 3), 2)
        total_mask = cv2.dilate(total_mask, (3, 3), 2)
        
        image_result = cv2.bitwise_and(img, img, mask=total_mask)
        
        self.scanner.set_image(total_mask)
        #d = self.scanner.find_direction()
        #p = Plotter.plot_direction(image_result, d)
        d = self.scanner.direction_exp()
        p = Plotter.plot_direction_exp(image_result, d)

        self.image = self.get_qimage(p)
        if self.image.size() != self.size():
            self.setFixedSize(self.image.size())
        self.update()

    def get_qimage(self, image): #image : np.ndarray
        height, width, colors = image.shape
        bytesPerLine = 3 * width

        image = QImage(image.data, width, height, bytesPerLine, QImage.Format_RGB888)
        image = image.scaledToWidth(self.maximumWidth())

        image = image.rgbSwapped()
        return image

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawImage(0, 0, self.image)
        self.image = QImage()