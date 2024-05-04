import numpy as np
import cv2 as cv
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from trimesh import Scene, load_mesh
from scipy.spatial.transform import Rotation as R


class TrackSimulator(Node):
    def __init__(self):
        super().__init__("track_simulator")

        # Simulate a camera going around the track
        self.image_pub = self.create_publisher(Image, "/track_camera", 10)
        self.timer = self.create_timer(1 / 15, self.timer_cb)
        self.bridge = CvBridge()

        self.scene = Scene(load_mesh("track.glb"))
    
    def timer_cb(self):
        self.scene.camera.z_near = 1e-3

        pose = np.eye(4)
        pose[:3, :3] = R.from_quat([0.469, -0.454, -0.533, 0.538]).as_matrix()
        pose[:3, 3] = [-60, 32, 2]
        self.scene.camera_transform = pose

        png = self.scene.save_image((1920, 1080))
        img = cv.imdecode(png, cv.IMREAD_COLOR)
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")

        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(TrackSimulator())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()