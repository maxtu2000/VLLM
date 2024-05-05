#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
from actionlib import SimpleActionServer
from control_msgs.msg import FollowJointTrajectoryAction
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import time
'''
pose: 
  position: 
    x: 0.030209544765586543
    y: 0.0
    z: 0.20979718610423298
  orientation: 
    x: 0.0
    y: 0.12925868509368116
    z: 0.0
    w: 0.9916109077293637
'''

JOINT_TOLERANCE = 0.05
GRIPPER_TOLERANCE = 0.01
POSITION_TOLERANCE = 0.02

def pose_to_list(pose_msg):
    pose = []
    pose.append(pose_msg.position.x)
    pose.append(pose_msg.position.y)
    pose.append(pose_msg.position.z)
    return pose

def all_close(goal, actual, tolerance):
    
    if type(goal) is list:
        for i in range(len(goal)):
            if abs(actual[i] - goal[i]) > tolerance:
                # print(abs(actual[i] - goal[i]))
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    
    return True

class Arm:
    def __init__(self):
        self.arm_action_server = SimpleActionServer(
            "arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.armActionCallback,
            auto_start=False
        )
        self.gripper_action_server = SimpleActionServer(
            "gripper_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.gripperActionCallback,
            auto_start=False
        )
        self.joint_trajectory_point_pub = rospy.Publisher("joint_trajectory_point", Float64MultiArray, queue_size=10)
        self.gripper_pub = rospy.Publisher("gripper_position", Float64MultiArray, queue_size=10)
        self.joint_position_sub = rospy.Subscriber('/joint_states', JointState, self.jointStatesCallback)
        self.arm_action_server.start()
        self.gripper_action_server.start()

        self.joint_position = None
        self.controller = None
        self.gripper_effort = 0

        rospy.loginfo(('\033[95m' + " >>> Arm Bringup is done." + '\033[0m'))

        try:
            rospy.wait_for_service('/get_planning_scene', timeout=10)
            self.controller = Controller()
        except rospy.ROSException:
            rospy.signal_shutdown('Can not initialize the arm controller')
            print(f"Service ['/get_planning_scene'] has not been advertised, waiting...")


    def jointStatesCallback(self, msg):
        self.joint_position = msg.position[2:6]
        self.gripper_effort = msg.effort[-1]

    def armActionCallback(self, goal):
        jnt_tra = goal.trajectory
        jnt_tra_pts = Float64MultiArray()

        jnt_tra_pts_size = len(jnt_tra.points)
        print(jnt_tra_pts_size)
        POINTS_STEP_SIZE = 10   
        steps = int(jnt_tra_pts_size / POINTS_STEP_SIZE)
        
        if steps != 0:
            for i in range(0, jnt_tra_pts_size, steps):
                jnt_tra_pts.data.append(jnt_tra.points[i].time_from_start.to_sec())
                for j in range(len(jnt_tra.points[i].positions)):
                    jnt_tra_pts.data.append(jnt_tra.points[i].positions[j])

            self.joint_trajectory_point_pub.publish(jnt_tra_pts)
            goal_position = jnt_tra_pts.data[-4:]
            time_sleep = 0
            while not all_close(goal=goal_position, actual=self.joint_position, tolerance=JOINT_TOLERANCE):
                rospy.sleep(0.1)
                time_sleep+=0.1
                if time_sleep > 10:
                    break

        self.arm_action_server.set_succeeded()
        rospy.loginfo('\033[32m' + "Succeed move to the target position"+ '\033[0m')

            
    def gripperActionCallback(self, goal):
        jnt_tra = goal.trajectory
        jnt_tra_pts = Float64MultiArray()

        jnt_tra_pts_size = len(jnt_tra.points)
        jnt_tra_pts.data.append(jnt_tra.points[jnt_tra_pts_size - 1].positions[0])
        self.gripper_pub.publish(jnt_tra_pts)
        # if jnt_tra_pts.data[-1]<0:
        #     while self.gripper_effort<150:
        #         pass
        #     self.arm_controller.grip.stop()
        self.gripper_action_server.set_succeeded()
        if jnt_tra_pts.data[-1]>0:
            rospy.loginfo('\033[32m' + "Successfully open the gripper"+ '\033[0m')
        else:
            rospy.loginfo('\033[32m' + "Successfully close the gripper"+ '\033[0m')



class Controller:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"  
        griper_name = "gripper"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.grip = moveit_commander.MoveGroupCommander(griper_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        # self.exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        # self.exectute_trajectory_client.wait_for_server()
        self.group.set_max_velocity_scaling_factor(1)
        # self.group.set_planning_time(5) 
        # self.grip.set_planning_time(2) 

        self.planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % self.planning_frame)

        self.eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % self.eef_link)

        self.group_names = self.robot.get_group_names()
        print("============ Robot Groups:", self.group_names)

        self.home_pose = self.group.get_current_pose()
        self.home_joint = [0.0, -1.6122138500213623, 1.2486604452133179, 0.6550098061561584]
        self.hold_joint = [0.0, -1.57, 0.96, 0.6]
        self.place_joint = [math.pi/2,0.0,0.0,0.0]
        self.front_joint = [0.0,0.0,0.0,0.0]
        self.joint3 = [math.pi/2,0.0,0.0,math.pi/2]

        self.grab_pose = self.home_pose
        self.grab_pose.pose.position.x = 0.0
        self.grab_pose.pose.position.y = 0.0
        self.grab_pose.pose.position.z = 0.14
        self.grab_pose.pose.orientation.x = 0.0
        self.grab_pose.pose.orientation.y = 0.0
        self.grab_pose.pose.orientation.z = 0.0
        self.grab_pose.pose.orientation.w = 1.0

        self.pick_depth = 0.0

    def set_gripper(self,arg):
        # open: 0.01 close: -0.001
        joint_grip = self.grip.get_current_joint_values()
        joint_grip[0] = arg
        self.grip.go(joint_grip, wait=True)
        self.grip.stop()

    def move_arm(self,arg):
        if type(arg) is list:
            joint_goal = self.group.get_current_joint_values()
            for i in range(len(arg)):
                joint_goal[i] = arg[i]
            self.group.go(joint_goal, wait=True)
            self.group.stop()

        elif type(arg) is float:
            pose_goal = self.grab_pose
            pose_goal.pose.position.x = arg+0.03
            # print(pose_goal)
            self.group.set_pose_target(pose_goal)
            self.group.go(wait=True)
            self.group.stop()

    def pick(self,depth):
        self.pick_depth = depth
    
        self.move_arm(depth)
        time.sleep(0.5)
        self.set_gripper(0.005)
        time.sleep(0.5)
        self.move_arm(self.hold_joint)
        time.sleep(1)


        # time.sleep(0.5)
        # self.move_arm(depth)
        # time.sleep(0.5)
        # self.set_gripper(0.015)
        # time.sleep(0.5)
        # self.move_arm(self.hold_joint)
        # time.sleep(0.5)

        # self.move_arm(self.joint3)
        # print(self.group.get_current_pose())

    def place(self):
    
        self.move_arm(self.pick_depth)
        time.sleep(0.5)
        self.set_gripper(0.015)
        time.sleep(0.5)
        self.move_arm(self.front_joint)
        time.sleep(0.5)
        self.move_arm(self.home_joint)
        time.sleep(1)
        



if __name__ == '__main__':
    try:
        rospy.init_node('Arm_node')
        arm = Arm()
        if arm.controller:
            arm.controller.pick(0.22)
            # arm.controller.place()
        # controller = Controller()
        # controller.pick(0.1859)

    except rospy.ROSInterruptException:
        pass
