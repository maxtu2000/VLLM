#!/usr/bin/env python3
import rospy
import cv2
import pyrealsense2 as rs
from pyrealsense2 import composite_frame, align
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
import numpy as np
class RGB_D:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        rospy.loginfo('\033[32m' + "[INFO] Starting streaming…"+ '\033[0m')
        self.pipeline.start(config)
        rospy.loginfo('\033[32m' + "[INFO] Camera ready"+ '\033[0m')
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.bridge = CvBridge()
        self.depth_intrin_pub = rospy.Publisher('/RGBD/depth_intrin', CameraInfo, queue_size=10)
        self.img_color_pub = rospy.Publisher('/RGBD/image/compressed', CompressedImage, queue_size=10)
        self.aligned_depth_frame_pub = rospy.Publisher('/RGBD/depth/compressed', CompressedImage, queue_size=10)

    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        img_color = np.asanyarray(aligned_color_frame.get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        return depth_intrin, img_color, depth_image

    def run(self):
        while not rospy.is_shutdown():
            depth_intrin, img_color, depth_image = self.get_aligned_images()
            cv2.imshow("color", img_color)
            cv2.imshow("depth", depth_image)
            print(depth_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rospy.signal_shutdown("q")
            # print(depth_image)
            # depth_intrin_msg = CameraInfo()
            # depth_intrin_msg.header.stamp = rospy.Time.now()
            # depth_intrin_msg.header.frame_id = "camera_color_optical_frame"
            # depth_intrin_msg.width = depth_intrin.width
            # depth_intrin_msg.height = depth_intrin.height
            # depth_intrin_msg.height = depth_intrin.height
            # depth_intrin_msg.K = [depth_intrin.fx, 0, depth_intrin.ppx, 0, depth_intrin.fy, depth_intrin.ppy, 0, 0, 1]
            # depth_intrin_msg.P = [depth_intrin.fx, 0, depth_intrin.ppx, 0, 0, depth_intrin.fy, depth_intrin.ppy, 0, 0, 0, 1, 0]
            # depth_intrin_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            # img_color_msg = self.bridge.cv2_to_compressed_imgmsg(img_color)
            # aligned_depth_frame_msg = self.bridge.cv2_to_compressed_imgmsg(depth_image)
            # self.depth_intrin_pub.publish(depth_intrin_msg)
            # self.img_color_pub.publish(img_color_msg)
            # self.aligned_depth_frame_pub.publish(aligned_depth_frame_msg)


class Camera:
    def __init__(self):
        pass
        # self.image_subscriber = rospy.Subscriber('/RGBD/image', CompressedImage, self.image_callback)
        # self.depth_subscriber = rospy.Subscriber('/RGBD/depth', CompressedImage, self.depth_callback)

    def run(aself):
        try:
            while not rospy.is_shutdown():
                image_msg = rospy.wait_for_message('/RGBD/image', CompressedImage)
                depth_msg = rospy.wait_for_message('/RGBD/depth', CompressedImage)
                image_data = np.frombuffer(image_msg.data, np.uint8)
                depth_data = np.frombuffer(depth_msg.data, np.uint8)
                image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                depth = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)

                # frames = rs.composite_frame()
                # color_frame = rs.frame(image)
                # depth_frame = rs.frame(depth)
                # frames.add_frame(color_frame)
                # frames.add_frame(depth_frame)

                # aligned_frames = align.process(frames)
                # color = aligned_frames.get_color_frame()
                # depth = aligned_frames.get_depth_frame()

                # color_image = np.asanyarray(color.get_data())
                # depth_image = np.asanyarray(depth.get_data())

                cv2.imshow("Realsense Camera", image)
                cv2.waitKey(1)

        except ValueError as e:
            return

if __name__ == "__main__":
    rospy.init_node('camera_receiver')
    # camera = Camera()
    # camera.run()
    camera = RGB_D()
    camera.run()