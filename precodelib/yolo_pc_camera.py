from collections import defaultdict
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ultralytics.utils.plotting import Annotator, colors
from copy import deepcopy

current_path = os.path.dirname(os.path.abspath(__file__))
parent_path = os.path.dirname(current_path)
print(parent_path)
model_path = os.path.join(parent_path, 'weights/best(1).pt')

model = YOLO(model_path)

cap = cv2.VideoCapture(0)

track_history = defaultdict(lambda: [])
id_list = {}

while cap.isOpened():

    success, frame = cap.read()
    if success:
        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        max_confidence=0.8
        result = model.track(frame, persist=True, verbose=False)[0]
        pred_boxes = result.boxes
        # track_ids = [int(x) for x in pred_boxes.id.tolist()] 
        # print(track_ids)
        img_in = result.orig_img
        font='Arial.ttf'
        names = model.names
        annotator = Annotator(
                im=deepcopy(img_in),
                font=font,
                example=names)
        id_selected =[]
        d_selected = []
        for d in pred_boxes:
            conf = float(d.conf)
            if conf > max_confidence:
                d_selected.append(d)
                id_selected.append(str(int(d.id[0])))
        print("id_selected:", id_selected)

        if d_selected:
            for d in d_selected:
                c, conf, id = int(d.cls), float(d.conf), None if d.id is None else int(d.id.item())
                name = ('' if id is None else f'id:{id} ') + 'bottle'
                label = (f'{name} {conf:.2f}' if conf else name)
                max_box = d.xyxy.squeeze().tolist()
                annotator.box_label(d.xyxy.squeeze(), label, color=colors(c, True))

        ids = {}
        for d in pred_boxes:
            conf = float(d.conf)
            if conf > max_confidence:
                id = str(int(d.id.tolist()[0]))
                ids[id] = [d, 1]
        for id in ids.keys():
            if id in id_list:
                ids[id][1] += id_list[id][1]

        id_list = ids

        max_d = None
        max_conf = 0
        for id, value in id_list.items():
            if value[1]>50:
                conf = float(value[0].conf)
                if conf > max_conf:
                    max_conf =conf
                    max_d = value[0]
        if max_d:
            print("[INFO] RGBD Get a new target. Target position publish is done.")
            id_list.clear()


        # print(id_list)
        img_out = annotator.result()
        # Get the boxes and track IDs
        # boxes = results[0].boxes.xywh

        # track_ids = results[0].boxes.id
        # list_int = [int(x) for x in track_ids.tolist()]

        # # Visualize the results on the frame
        # annotated_frame = results[0].plot()
        # print(list_int)

        # Plot the tracks
        # for box, track_id in zip(boxes, track_ids):
        #     x, y, w, h = box
        #     track = track_history[track_id]
        #     track.append((float(x), float(y)))  # x, y center point
        #     if len(track) > 30:  # retain 90 tracks for 90 frames
        #         track.pop(0)

        #     # Draw the tracking lines
        #     points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
        #     cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)

        # Display the annotated frame
        cv2.imshow("YOLOv8 Tracking", img_out)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()