#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import tf
import math

'''
header: 
  seq: 3
  stamp: 
    secs: 1710136167
    nsecs: 703729749
  frame_id: "map"
pose: 
  position: 
    x: -3.5639476776123047
    y: 4.020162582397461
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.6572121513983169
    w: 0.7537056375365623


'''

class Navigator:
    def __init__(self):

        self.tf_listener = tf.TransformListener()
        self.bot_status = 0
        self.send_target = False
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        # self.enable_imgCb_pub = rospy.Publisher("/tarsh_bot/imgCb_status", bool, queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.home = MoveBaseGoal()
        self.home.target_pose.header.frame_id='map'
        
        self.home.target_pose.pose.position.x = -3.5639476776123047
        self.home.target_pose.pose.position.y = 4.020162582397461
        self.home.target_pose.pose.orientation.z = 0.6572121513983169
        self.home.target_pose.pose.orientation.w = 0.7537056375365623
        self.goal = self.home


    # def coordinate_callback(self,msg):
    #     self.goal = [msg.x,msg.y, self.goal[2]]
    #     self.send_goal_through_client()
    
    def get_robot_position(self):
        try:
            self.tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            return trans,rot
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to get robot's position.")
            return None

    def send_goal_through_client(self, status):

        print(self.goal)
        self.client.send_goal(self.goal)

        if status:
            wait = self.client.wait_for_result(rospy.Duration.from_sec(120.0))
            if not wait:
              rospy.loginfo('\033[32m' + "[INFO] The Goal Planning Failed for some reasons"+ '\033[0m')
              return False
            else:
                rospy.loginfo('\033[32m' + "[INFO] The Goal achieved success"+ '\033[0m')
                # self.enable_imgCb_pub.publish(True)
                speed_msg = Twist()
                self.vel_pub.publish(speed_msg)
                return True
        else:
          timeout = rospy.Duration.from_sec(120.0)
          timeout_time = rospy.get_rostime() + timeout
          trash_pos = [self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y]
          while True:
              time_left = timeout_time - rospy.get_rostime()
              if timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                rospy.loginfo('\033[32m' + "[INFO] The Goal Planning Failed for some reasons"+ '\033[0m')
                return False
              
              robot_pos, _ = self.get_robot_position()
              dis = self.calculate_distance([robot_pos[0],robot_pos[1]],trash_pos)
              print(dis)
              if dis < 0.5:
                  self.client.cancel_goal()
                  speed_msg = Twist()
                  self.vel_pub.publish(speed_msg)
                  rospy.loginfo('\033[32m' + "[INFO] The Goal achieved success"+ '\033[0m')
                  return True
        

    def send_goals_through_topic(self):
        goal = PoseStamped()
        goal.header.frame_id='map'
        goal.pose.position.x = 0.5899236798286438
        goal.pose.position.y = 1.3233416080474854
        goal.pose.orientation.z = 0.8596124084983308
        goal.pose.orientation.w = 0.5109466774093936
        self.goal_pub.publish(goal)

    def calculate_distance(self, loc_a, loc_b):
        distance = math.sqrt((loc_b[0] - loc_a[0]) ** 2 + (loc_b[1] - loc_a[1]) ** 2)
        return distance

if __name__ == '__main__':
    rospy.init_node('navigator')
    navigator = Navigator()
    # navigator.send_goal_through_client()
    print(navigator.get_robot_position())
    rospy.spin()