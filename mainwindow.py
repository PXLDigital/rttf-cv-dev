from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QHBoxLayout, QVBoxLayout, QPushButton
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from camera_reader import CameraReader
from camview import CamView
from filterview import FilterView
import cv2

class MainWindow(QWidget):
    def __init__(self, title, parent=None):
        super(QWidget, self).__init__(parent)
        self.camera = CameraReader()

        self.setWindowTitle(title)
        self.mainlayout = QHBoxLayout()

        self.camview = CamView()
        self.camview.setFixedWidth(600)
        self.camera.subscribe(self.camview.image_data_slot)

        self.filterview = FilterView()
        self.filterview.setFixedWidth(600)
        self.camera.subscribe(self.filterview.image_data_slot)

        self.mainlayout.addWidget(self.camview)
        self.mainlayout.addWidget(self.filterview)
        self.buttons = QVBoxLayout()
        
        self.mainlayout.addLayout(self.buttons)

        # Default: live image
        self.btn_image_pause = QPushButton('Pause')
        self.btn_image_pause.clicked.connect(self.pause)
        self.buttons.addWidget(self.btn_image_pause)

        self.btn_image_resume = QPushButton('Resume')
        self.btn_image_resume.clicked.connect(self.resume)
        self.buttons.addWidget(self.btn_image_resume)
        
        self.setLayout(self.mainlayout)

    def pause(self):
        self.camera.skip(True)

    def resume(self):
        self.camera.skip(False)