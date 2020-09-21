import numpy as np
import cv2
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb

class Plotter:
    
    @staticmethod
    def plot_rgb(img):
        r, g, b = cv2.split(img)
        fig = plt.figure()
        axis = fig.add_subplot(1, 1, 1, projection="3d")

        pixel_colors = Plotter.get_pixelcolors(img)

        axis.scatter(r.flatten(), g.flatten(), b.flatten(), facecolors=pixel_colors, marker=".")
        axis.set_xlabel("Red")
        axis.set_ylabel("Green")
        axis.set_zlabel("Blue")
        plt.show()

    @staticmethod
    def plot_hsv(img):
        hsv_data = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        h, s, v = cv2.split(hsv_data)
        fig = plt.figure()
        axis = fig.add_subplot(1, 1, 1, projection="3d")

        pixel_colors = Plotter.get_pixelcolors(img)

        axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
        axis.set_xlabel("Hue")
        axis.set_ylabel("Saturation")
        axis.set_zlabel("Value")
        plt.show()

    @staticmethod
    def get_pixelcolors(img):
        pixel_colors = img.reshape((np.shape(img)[0]*np.shape(img)[1], 3))
        norm = colors.Normalize(vmin=-1.,vmax=1.)
        norm.autoscale(pixel_colors)
        return norm(pixel_colors).tolist()

    @staticmethod
    def plot_img(img):
        cv2.imshow('image',img)
        cv2.waitKey(0)
        #cv2.destroyAllWindows()

    @staticmethod
    def plot_colors_hsv(lower, upper):
        lo_square = np.full((10, 10, 3), lower, dtype=np.uint8) / 255.0
        up_square = np.full((10, 10, 3), upper, dtype=np.uint8) / 255.0

        plt.subplot(1, 2, 1)
        plt.imshow(hsv_to_rgb(up_square))
        plt.subplot(1, 2, 2)
        plt.imshow(hsv_to_rgb(lo_square))
        plt.show()

    @staticmethod
    def plot_direction(img, result):
        height = len(img)
        width = len(img[0])

        (direction, mid, current_row, left, right) = result

        cv2.line(img, (int(width/2), height-1), (mid, current_row), (0, 0, 255), thickness=2) # direction line

        cv2.line(img, (0, current_row), (width, current_row), (0, 255, 255), thickness=1)
        cv2.line(img, (mid, height-1), (mid, 0), (255, 0, 0), thickness=1)

        cv2.line(img, (int(width/2), height-1), (int(width/2), 0), (255, 0, 0), thickness=1)

        cv2.rectangle(img, (left-5, current_row-5), (left+5, current_row+5), (0,255,255))
        cv2.rectangle(img, (right-5, current_row-5), (right+5, current_row+5), (0,255,255))

        return img

    @staticmethod
    def plot_direction_exp(img, result):
        height = len(img)
        width = len(img[0])

        (target, absdir, left, right) = result

        cv2.line(img, (int(width/2), height-1), (target[1], target[0]), (0, 0, 255), thickness=2) # direction
        cv2.line(img, (int(width/2), height-1), (int(width/2), 0), (255, 0, 0), thickness=1) # mid        

        cv2.rectangle(img, (-5, left-5), (5, left+5), (0,255,255))
        cv2.rectangle(img, (width-5, right-5), (width+5, right+5), (0,255,255))
        
        return img