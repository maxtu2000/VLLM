#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

rospy.init_node('init_pos')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)

rospy.sleep(3)
checkpoint = PoseWithCovarianceStamped()

checkpoint.pose.pose.position.x = 0.006610089598103543
checkpoint.pose.pose.position.y = 1.0000051003430117
checkpoint.pose.pose.position.z = -0.0005050488616139027

checkpoint.pose.pose.orientation.x = -0.0016972314084476505
checkpoint.pose.pose.orientation.y = 0.0007621281777511023
checkpoint.pose.pose.orientation.z = 0.0007223646351435822
checkpoint.pose.pose.orientation.w = 0.9999980083757771

print(checkpoint)
pub.publish(checkpoint)





