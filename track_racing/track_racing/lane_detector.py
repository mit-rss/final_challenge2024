import numpy as np
import cv2 as cv
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from skimage.morphology import binary_dilation, binary_erosion, skeletonize, rectangle, square
from math import pi, cos, sin, atan2


# Pixels
HOMOGRAPHY_IMAGE_PLANE = [
    [364, 175],
    [623, 170],
    [585, 217],
    [133, 222],
]
# Inches (+x forward, +y left)
HOMOGRAPHY_GROUND_PLANE = [
    [132.5, 3],
    [122, -97],
    [33.5, -25],
    [35, 35],
]
METERS_PER_INCH = 0.0254

class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        self.simulation = self.declare_parameter("simulation", True).get_parameter_value().bool_value
        self.camera_topic = "/track_camera" if self.simulation else "/zed/zed_node/rgb/image_rect_color"
        
        # Subscription to the ZED2 camera
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_cb, 5)
        self.bridge = CvBridge()

        # Publisher for debugging CV stuff
        self.debug_image_pub = self.create_publisher(Image, "/lane_debug_img", 10)

        # Homography maps 2D lanes to 3D
        self.homography, _ = cv.findHomography(
            np.array(HOMOGRAPHY_GROUND_PLANE)[:, np.newaxis, :] * METERS_PER_INCH,
            np.array(HOMOGRAPHY_IMAGE_PLANE)[:, np.newaxis, :],
        )

        self.log("Lane detector initialized.")
    
    def log(self, s):
        """Short-hand for logging messages"""
        self.get_logger().info(s)
    
    def image_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        viz, rho, theta = LaneDetector.find_lanes(img, sim=self.simulation)
        lines = LaneDetector.merge_lines(rho, theta)

        # Visualize
        for (rho, theta, _) in lines:
            cv.line(viz, *LaneDetector.line_polar_to_cartesian(rho, theta), (255, 0, 0), 1, cv.LINE_AA)
        
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(viz, "bgr8"))
    
    @staticmethod
    def find_lanes(img, *, sim=False):
        """
        Computer-vision algorithm to detect lanes on a track. Uses polar representation.
        """
        # Down-sample image
        img = cv.resize(img, (600, 400), interpolation=cv.INTER_NEAREST)

        # Isolate the red track and lanes within it. This initially gets rid of the white lanes,
        # but we get them back by dilating the mask horizontally.
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, (0, 75, 100), (10, 200, 255) if sim else (10, 100, 180))
        mask = binary_dilation(mask, rectangle(5, 150 if sim else 50))
        img *= mask.astype(np.uint8)[:, :, np.newaxis]

        viz = np.copy(img)

        # Mask out the bright white lanes
        white = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img = np.greater(white, 150 if sim else 200).astype(np.uint8) * 255

        # Everything near top of image is probably not relevant and tends to leak
        img[:100, :] = 0

        # Erosion removes any noise
        img = binary_erosion(img, square(4))
        # Dilation gives the lines their original thickness and fixes any discontinuities
        img = binary_dilation(img, square(3))

        # Do a copy here so that visualization sees every CV stuff above here but not below
        # viz = cv.cvtColor(np.copy(img).astype(np.uint8) * 255, cv.COLOR_GRAY2BGR)

        # Reduce lines to 1-pixel wide representation
        img = skeletonize(img)
        img = img.astype(np.uint8) * 127

        # Reduce look-ahead
        img[:200, :] = 0

        # Find the lines in polar representation
        lines = cv.HoughLines(img, 1, pi / 180, 50, None, 0, 0)
        if lines is None:
            return (viz, [], [])
        return (viz, lines[:, 0, 0], lines[:, 0, 1])

    @staticmethod
    def merge_lines(rho, theta, *, rho_threshold=50, theta_threshold=10):
        """
        Merges lines that are close together. Uses polar representation. Returned is a
        list of tuples for (rho, theta, count) where count is the number of lines that
        were merged into this one.
        """
        # Normalize the lines (positive rho)
        rho, theta = np.array(rho), np.array(theta)
        theta[rho < 0] += pi
        rho[rho < 0] *= -1

        out = []

        # Iterate each line in O(n^2)
        for (r0, t0) in zip(rho, theta):
            for (i, (r1, t1, count)) in enumerate(out):
                # Lines can be merged if they are nearby and of similar slope
                if abs(r0 - r1) > rho_threshold:
                    continue
                if abs(atan2(sin(t0 - t1), cos(t0 - t1))) > (theta_threshold * (pi / 180)):
                    continue

                # Merge at the average
                rho = (r1 * count + r0) / (count + 1)
                theta = atan2(sin(t1) * count + sin(t0), cos(t1) * count + cos(t0))
                out[i] = (rho, theta, count + 1)
                break
            else:
                # No line was found to merge, so it goes through as-is
                out.append((r0, t0, 1))
        
        return out
    
    @staticmethod
    def line_polar_to_cartesian(rho, theta, *, dist=10000):
        """
        Converts a line from polar to cartesian representation.
        """
        # Source:
        # https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
        a = cos(theta)
        b = sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt0 = (int(x0 - (dist * b)), int(y0 + (a * dist)))
        pt1 = (int(x0 + (dist * b)), int(y0 - (a * dist)))

        return (pt0, pt1)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(LaneDetector())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()