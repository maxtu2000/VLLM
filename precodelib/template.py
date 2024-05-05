import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped


class ObjectGrabber:
    def __init__(self):
        rospy.init_node('object_grabber')

        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        rospy.Subscriber('object_pose', PoseStamped, self.object_pose_callback)

        self.initial_pose = rospy.get_param('~initial_pose')

        self.target_pose = None

    def object_pose_callback(self, pose):
        self.target_pose = pose

    def navigate_to_object(self):
        goal = MoveBaseGoal()
        goal.target_pose = self.target_pose
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

    def grab_object(self):
        rospy.sleep(1)

    def return_to_initial_pose(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = self.initial_pose
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

    def run(self):
        while not rospy.is_shutdown() and self.target_pose is None:
            rospy.sleep(0.1)
        self.navigate_to_object()
        self.grab_object()
        self.return_to_initial_pose()
