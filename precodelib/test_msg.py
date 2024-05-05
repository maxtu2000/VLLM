#!/usr/bin/env python3
import rospy
from trash_bot.msg import Frame, RealsenseData
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32MultiArray
bridge = CvBridge()

class Test:
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
        # self.depth_intrin_pub = rospy.Publisher('/RGBD/depth_intrin', CameraInfo, queue_size=10)
        self.data_pub = rospy.Publisher('/Realsense/data', Frame , queue_size=10)

    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        img_color = np.asanyarray(aligned_color_frame.get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())/100
        return depth_intrin, img_color, depth_image
    
    def run(self):
        while not rospy.is_shutdown():
            depth_intrin, img_color, depth_image = self.get_aligned_images()
            msg = Frame()
            
            msg.color_image = self.bridge.cv2_to_compressed_imgmsg(img_color)
            msg.depth_image = self.bridge.cv2_to_compressed_imgmsg(depth_image)
            depth_intrin_msg = CameraInfo()
            depth_intrin_msg.header.stamp = rospy.Time.now()
            depth_intrin_msg.header.frame_id = "camera_color_optical_frame"
            depth_intrin_msg.width = depth_intrin.width
            depth_intrin_msg.height = depth_intrin.height
            depth_intrin_msg.height = depth_intrin.height
            depth_intrin_msg.K = [depth_intrin.fx, 0, depth_intrin.ppx, 0, depth_intrin.fy, depth_intrin.ppy, 0, 0, 1]
            depth_intrin_msg.P = [depth_intrin.fx, 0, depth_intrin.ppx, 0, 0, depth_intrin.fy, depth_intrin.ppy, 0, 0, 0, 1, 0]
            depth_intrin_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.depth_intrin = depth_intrin_msg
            self.data_pub.publish(msg)

def data_callback(msg):

    # align = rs.align(rs.stream.color)

    # cv_image = bridge.compressed_imgmsg_to_cv2(msg.color_image)

    cv_image_depth = bridge.imgmsg_to_cv2(msg.depth_image)
    print(cv_image_depth)
    # frames = rs.composite_frame(cv_image, cv_image_depth)

    # aligned_frames = align.process(frames)
    # aligned_depth_frame = aligned_frames.get_depth_frame()
    # aligned_color_frame = aligned_frames.get_color_frame()
    # cv2.imshow("color", cv_image)
    cv2.imshow("depth", cv_image_depth)
    # print(cv_image_depth)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        rospy.signal_shutdown("q")


rospy.init_node('composite_frame_subscriber')
rospy.Subscriber('/RGBD/frame', Frame, data_callback)
# test = Test()
# test.run()
rospy.spin()