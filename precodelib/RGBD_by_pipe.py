#!/usr/bin/env python3
import rospy
from ultralytics import YOLO
import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics.utils.plotting import Annotator, colors
from copy import deepcopy
import os
from navigator import Navigator
from std_msgs.msg import Int16
from move_base_msgs.msg import MoveBaseGoal
import math
import tf.transformations as tf

current_path = os.path.dirname(os.path.abspath(__file__))
parent_path = os.path.dirname(current_path)
print(parent_path)

class RGB_D:
    def __init__(self):
        model_path = os.path.join(parent_path, 'weights/cola.pt')
        self.model = YOLO(model_path) 

        # self.navigator = Navigator()
        # self.coordinate_pub = rospy.Publisher('/trash_bot/trash_coordinate',Point,queue_size=1)
        self.old_coordinate = [0.0,0.0,0.0]
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        rospy.loginfo('\033[32m' + "[INFO] Starting streaming…"+ '\033[0m')
        self.pipeline.start(config)
        rospy.loginfo('\033[32m' + "[INFO] Camera ready"+ '\033[0m')
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.find_new_target = True

        self.translation = [[-0.988737,-0.31327,-0.109869, -0.42512],
                            [-0.02549,1.66143,0.038007,1.77671],
                            [0.0004719,0.0282502,0.01896,0.043183],
                            [0.0,0.0,0.0,1.0]]
        self.id_list = {}

        self.bot_status_pub = rospy.Publisher('/trash_bot/bot_status', Int16, queue_size=1)
        self.bot_status_sub = rospy.Subscriber("/trash_bot/bot_status", Int16, self.statusCallback)
        


    def statusCallback(self, msg):
        self.bot_status = msg
        status = msg.data
        if status == 0:
            self.find_new_target = True
        if status == 1:
            res = self.navigator.send_goal_through_client()
            if res:
                self.bot_status_pub.publish(2)
            else:
                self.bot_status_pub.publish(9)
        
        if status == 4:
            # send navigation goal to home position
            self.navigator.goal = self.navigator.home
            res = self.navigator.send_goal_through_client()
            if res:
                self.bot_status_pub.publish(6)
            else:
                self.bot_status_pub.publish(9)


    def isSameTarget(self,target):
        x = abs(target[0]- self.old_coordinate[0])
        y = abs(target[1]- self.old_coordinate[1])
        z = abs(target[2]- self.old_coordinate[2])
        if x>0.2 or y> 0.2 or z>0.2:
            return False
        
        return True
    
    def calNavGoal(self, trash_loc, robot_loc, r):
        trash_x = trash_loc[0]
        trash_y = trash_loc[1]
        robot_x = robot_loc[0]
        robot_y = robot_loc[1]

        dx = trash_x - robot_x
        dy = trash_y - robot_y


        length = math.sqrt(dx**2 + dy**2)
        dis = length - r

        normalized_dx = dx / length
        normalized_dy = dy / length

        x_goal = robot_x + normalized_dx * dis
        y_goal = robot_y + normalized_dy * dis

        orientation = math.atan2(normalized_dy, normalized_dx)

        quaternion = tf.quaternion_from_euler(0, 0, orientation)
        return x_goal, y_goal, quaternion

    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        img_color = np.asanyarray(aligned_color_frame.get_data())

        return depth_intrin, img_color, aligned_depth_frame
    
    def run(self):
        
        while not rospy.is_shutdown():
            depth_intrin, img_color, aligned_depth_frame = self.get_aligned_images()
            if self.find_new_target:
                max_confidence=0.85
                result = self.model(img_color,verbose=False)[0]
                pred_boxes = result.boxes
                img_in = result.orig_img
                font='Arial.ttf'
                names = self.model.names
                annotator = Annotator(
                        im=deepcopy(img_in),
                        font=font,
                        example=names)
                
                max_d = None
                for d in pred_boxes:
                    conf = float(d.conf)
                    if conf > max_confidence:
                        max_confidence = conf
                        max_d = d

                pix_x = 0.0
                pix_y = 0.0
                if max_d:
                    
                    c, conf, id = int(max_d.cls), float(max_d.conf), None if max_d.id is None else int(max_d.id.item())
                    name = ('' if id is None else f'id:{id} ') + 'bottle'
                    label = (f'{name} {conf:.2f}' if conf else name)
                    max_box = max_d.xyxy.squeeze().tolist()
                    annotator.box_label(max_d.xyxy.squeeze(), label, color=colors(c, True))
                    pix_x = int((max_box[0] + max_box[2]) / 2.0)
                    pix_y = int((max_box[1] + max_box[3]) / 2.0)

                    dis = aligned_depth_frame.get_distance(pix_x,pix_y)
                    trash_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [pix_x, pix_y], dis)
                    coordinate = [[trash_coordinate[0]],[trash_coordinate[1]],[trash_coordinate[2]],[1]]
                    coordinate = np.dot(self.translation,coordinate)
                    coordinate = coordinate.reshape(-1).tolist()
                    coordinate =coordinate[0:3]
                    print(coordinate)
                    # if trash_coordinate[0] == 0.0 and trash_coordinate[1]==0.0 and trash_coordinate[2]==0.0:    
                    #     pass
                    # else:

                    #     # target = Point()
                    #     # target.x = trash_coordinate[0]
                    #     # target.y = trash_coordinate[1]
                    #     # target.z = trash_coordinate[2]
                    #     if not self.isSameTarget(coordinate):
                    #         robot_position, _ = self.navigator.get_robot_position()
                    #         goal_x, goal_y, quaternion = self.calNavGoal(coordinate,robot_position,0.3)
                    #         goal = MoveBaseGoal()
                    #         goal.target_pose.header.frame_id='map'
                    #         goal.target_pose.pose.position.x = goal_x
                    #         goal.target_pose.pose.position.y = goal_y
                    #         goal.target_pose.pose.orientation.z = quaternion[2]
                    #         goal.target_pose.pose.orientation.w = quaternion[3]
                    #         self.navigator.goal = goal
                            
                    #         self.old_coordinate = coordinate
                    #         self.find_new_target = False
                    #         self.bot_status_pub.publish(1)
                    #         rospy.loginfo('\033[32m' + "[INFO] RGBD Get a new target. Target position publish is done."+ '\033[0m')
                    #     else:
                    #         self.old_coordinate = coordinate
                    # # print(coordinate)
                img_out = annotator.result()
                cv2.imshow("Realsense Camera", img_out)
                cv2.waitKey(1)
            
            cv2.imshow("Realsense Camera", img_color)
            cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node('RGBD')
    camera = RGB_D()
    camera.run()
    