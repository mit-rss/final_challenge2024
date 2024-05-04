import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
from stop_msgs.msg import PixelLocation
from detector import StopSignDetector

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        self.detector = StopSignDetector()
        self.publisher = self.create_publisher(PixelLocation, "/relative_sign_px", 10)
        self.debug_pub = self.create_publisher(Image, "/stopsign_debug_img", 10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Stop Detector Initialized")

    def callback(self, img_msg):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        #jaclyn's stuff
        isStopSign, bounding_box = self.detector.predict(image)

        #from bounding box compute u,v (local position of stop sign)

        pixel_msg = PixelLocation()
        #send message only when stop sign is detected

        if isStopSign:
            cv2.rectangle(image, (bounding_box[0], bounding_box[1]), (bounding_box[2], bounding_box[3]), (0, 255, 0), 2)
            if (bounding_box[0] + bounding_box[1] + bounding_box[2] + bounding_box[3]) !=0:
                sign_height = bounding_box[3]-bounding_box[1]
                bot_y = bounding_box[3]+sign_height #estimate sign to be two stop sign faces high, location at base of sign
                center_bot_x = int((bounding_box[0]+bounding_box[2])/2)

                pixel_msg = PixelLocation()
                pixel_msg.u = float(center_bot_x)
                pixel_msg.v = float(bot_y)

                self.publisher.publish(pixel_msg)

            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()