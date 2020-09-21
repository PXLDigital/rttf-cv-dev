from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QImage, QPainter

class CamView(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)
        self.image = QImage()

    def image_data_slot(self, image_data, raw):
        self.image = self.get_qimage(image_data)
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