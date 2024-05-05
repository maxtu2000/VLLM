#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from copy import deepcopy
import os
import pyrealsense2 as rs2
import cv2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2


current_path = os.path.dirname(os.path.abspath(__file__))
parent_path = os.path.dirname(current_path)
print(parent_path)

class ImageListener:
    def __init__(self):
        self.bridge = CvBridge()
       
        model_path = os.path.join(parent_path, 'weights/best1.pt')
        self.model = YOLO(model_path) 
        self.depth_image = None
        self.intrinsics = []
        self.pix = None
        self.send_target = False

        self.img_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.imgColorCallback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', msg_Image, self.depthCallback)
        self.depth_sub_info = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.depthInfoCallback)

    def imgColorCallback(self,data):
        try:
            max_box = None
            max_confidence = 0.0
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
            result = self.model(cv_image,verbose=False)[0]
            pred_boxes = result.boxes
            img_in = result.orig_img
            font='Arial.ttf'
            names = self.model.names
            annotator = Annotator(
                    im=deepcopy(img_in),
                    font=font,
                    example=names)
            if len(pred_boxes)<1:
                self.send_target = False
            else:
                self.send_target = True
            
            max_d = None
            for d in pred_boxes:
                conf = float(d.conf)
                if conf > max_confidence:
                    max_confidence = conf
                    max_d = d
            
            if max_d:
                c, conf, id = int(max_d.cls), float(max_d.conf), None if max_d.id is None else int(max_d.id.item())
                name = ('' if id is None else f'id:{id} ') + 'bottle'
                label = (f'{name} {conf:.2f}' if conf else name)
                max_box = max_d.xyxy.squeeze().tolist()
                annotator.box_label(d.xyxy.squeeze(), label, color=colors(c, True))

            img_out = annotator.result()

            if max_box is not None and self.depth_image is not None:
                
                pix_x = int((max_box[0] + max_box[2]) / 2.0)
                pix_y = int((max_box[1] + max_box[3]) / 2.0)
                pix = (pix_x,pix_y)
                depth = self.depth_image[pix[1], pix[0]]
                if self.intrinsics:
                    coordinate = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                    if coordinate[2]==0.0:
                        print("depth of the object=", depth)
                    print(coordinate)
                cv2.circle(img_out, (pix_x,pix_y), 3, [255, 0, 255], thickness=-1)

            cv2.imshow("Image", img_out)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def depthCallback(self,data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
            
    def depthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]

            print(self.intrinsics)
        except CvBridgeError as e:
            print(e)
            return

def main():
    
    listener = ImageListener()
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('realsense_listener')
    main()