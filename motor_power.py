#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
rospy.init_node('motor_power_node')
pub = rospy.Publisher('/motor_power', Bool, queue_size = 10)

rospy.sleep(3)
checkpoint = True
pub.publish(checkpoint)



