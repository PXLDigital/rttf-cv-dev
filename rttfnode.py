def process_img(data):
    DEFAULT_ANGLE = 0.3
    DEFAULT_THROTTLE = 0.2

    img = self.bridge.compressed_imgmsg_to_cv2(data)
    hsv_data = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    scanner = Scanner()
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
    (target, absdir, left, right) = self.scanner.direction_exp()
    #p = Plotter.plot_direction_exp(image_result, d)

    angle = DEFAULT_ANGLE * absdir
    throttle = DEFAULT_THROTTLE

    #SEND
    msg = Joy() # IF Joy type
    msg.axes = [0.0, self.vx, 0.0, self.vy, 0.0, 0.0, 0.0, 0.0]
    pub.publish(msg)

def if __name__ == "__main__":
    rospy.init_node('RttfCameraListener', anonymous=True)
    rospy.Subscriber("/master/camera_node/image/compressed", CompressedImage, self.process_img, queue_size=1, buff_size=2**24)

    pub = rospy.Publisher("/master/......", Joy, queue_size=1)