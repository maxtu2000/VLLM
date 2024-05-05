#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import cv2
import math
from std_msgs.msg import Float32

def lidar_callback(data):    
    ranges = data.ranges
    ranges = np.concatenate((ranges[180:], ranges[:180]))
    filtered_values = [x for x in ranges if x != math.inf]
    
    min_distance = min(filtered_values)
    max_distance = max(filtered_values)
    
    
    pixel_values = np.interp(ranges, (min_distance, max_distance, math.inf), (10, 240,255))
    print(pixel_values)
    print("=======================")
    print(pixel_values.shape)
    image = np.zeros((50, 360), dtype=np.uint8)
    for i in range(360):
        image[:,i] = pixel_values[359-i]

    cv2.imshow("Lidar Image", image)
    cv2.waitKey(1)

def depth_callback(msg):
    print("Got depth message, depth = ",msg)

# def scan_callback(scan_data):
#     ranges = scan_data.ranges
#     plt.clf()  
#     angles = np.linspace(-np.pi, np.pi, len(ranges), endpoint=False)
#     ranges = np.concatenate((ranges[180:], ranges[:180]))

#     plt.bar(np.rad2deg(angles), ranges, width=np.rad2deg(angles[1]-angles[0]), align='center')
#     # plt.xlabel('angle')
#     # plt.ylabel('distance')
#     plt.draw()
#     plt.pause(0.1)

#     # plt.show()

if __name__ == '__main__':
    try:
        rospy.init_node('lidar_visualization_node', anonymous=True)
        rospy.loginfo("Starting node")
        # rospy.Subscriber('/scan', LaserScan, lidar_callback)
        rospy.Subscriber('/depth', Float32, depth_callback)
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

