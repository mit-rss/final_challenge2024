import numpy as np
import cv2 as cv
import rclpy
import os
import trimesh

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyrender import OffscreenRenderer, Scene, PerspectiveCamera, DirectionalLight
from scipy.spatial.transform import Rotation as R

os.environ["PYOPENGL_PLATFORM"] = "egl"

class TrackSimulator(Node):
    def __init__(self):
        super().__init__("track_simulator")

        # Simulate a camera going around the track
        self.image_pub = self.create_publisher(Image, "/track_camera", 10)
        self.timer = self.create_timer(1 / 15, self.timer_cb)
        self.bridge = CvBridge()

        self.mesh = trimesh.load("/home/racecar/racecar_ws/src/final_challenge2024/track_racing/map/track.glb")
        self.scene = Scene.from_trimesh_scene(self.mesh)
        self.camera = self.scene.add(PerspectiveCamera(yfov=72 * np.pi / 180, znear=1e-3), pose=np.eye(4))
        self.light = self.scene.add(DirectionalLight(color=np.ones(3), intensity=5.0), pose=np.eye(4))
        self.renderer = OffscreenRenderer(1920, 1080)

        self.log("Track simulator started!")
    
    def log(self, s):
        self.get_logger().info(s)

    def timer_cb(self):
        pose = np.eye(4)
        pose[:3, :3] = R.from_quat([0.469, -0.454, -0.533, 0.538]).as_matrix()
        pose[:3, 3] = [-60, 32, 4]

        self.scene.set_pose(self.camera, pose)
        self.scene.set_pose(self.light, pose)

        img, _ = self.renderer.render(self.scene)
        img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
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