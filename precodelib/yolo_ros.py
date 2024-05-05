#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from trash_bot.msg import Frame
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from cv_bridge import CvBridge
from copy import deepcopy
import cv2
import os

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 0.5
current_path = os.path.dirname(os.path.abspath(__file__))
parent_path = os.path.dirname(current_path)
print(parent_path)

class Detector:
    def __init__(self):
        rospy.init_node('trash_detector')
        self.img_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.imgCallback)
        self.bridge = CvBridge()
        models_path = os.path.join(parent_path, 'weights/cola_bot.pt')
        self.model = YOLO(models_path) 
        self.target_class = 'trash'
        self.count = 0


    def imgCallback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            result = self.model(cv_image,verbose=False)[0]
            pred_boxes = result.boxes
            img_in = result.orig_img
            font='Arial.ttf'
            names = self.model.names
            annotator = Annotator(
                    im=deepcopy(img_in),
                    font=font,
                    example=names)

            for d in pred_boxes:
                c, conf, id = int(d.cls), float(d.conf), None if d.id is None else int(d.id.item())
                name = ('' if id is None else f'id:{id} ') + names[c]
                label = (f'{name} {conf:.2f}' if conf else name)
                annotator.box_label(d.xyxy.squeeze(), label, color=colors(c, True))

            img_out = annotator.result()
            cv2.imshow("Image", img_out)
            time = rospy.Time.now()
            k = cv2.waitKey(1)
            # if k == ord('s'):
            #     self.count+=1
            #     name = '/home/face/catkin_ws/src/trash_bot/image/Image-'+ str(self.count)+'-' + str(time)+'.jpg'
            #     print(name)
            #     cv2.imwrite(name, cv_image)

        except Exception as e:
            rospy.logerr(e)

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = Detector()
    detector.run()






