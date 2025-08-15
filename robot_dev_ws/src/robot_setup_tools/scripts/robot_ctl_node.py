import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from yunhai_msgs.srv import SendGoal, SendGoalRequest
from franka_msgs.msg import FrankaState
from franka_msgs.srv import GetTFEcho, GetTFEchoRequest

class RobotCtrlNode():
    def __init__(self):
        rospy.loginfo('ArmCtrlNode has been started!')
        self.pub_nav_goal = rospy.Publisher('/msg', Pose, queue_size=1)
        self.pub_arm_pick_pose = rospy.Publisher('/pick_pose', Pose, queue_size=1)
        self.pub_arm_place_pose = rospy.Publisher('/place_pose', Pose, queue_size=1)

        self.nav_to_goal_cli = rospy.ServiceProxy('/yunhai_go_pose', SendGoal)
        self.get_cur_arm_pose_cli = rospy.ServiceProxy('/tf_echo_server', GetTFEcho)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.on_arm_pose_msg)
        self.arm_pose = Pose()


    def on_arm_pose_msg(self, msg: FrankaState):
        self.arm_joints = msg.q

    def get_cur_arm_pose(self):
        req = GetTFEchoRequest()
        req.parent_link = 'pand_link0'
        req.child_link = 'pand_link8'

        res = self.get_cur_arm_pose_cli.call(req)

        return req


    def nav_to_goal(self, x:float, y:float, yaw: float, use_coord_nav:bool):
        goal = Pose()
        rospy.loginfo(f'navigation goal has been published, goal: x: {x}, y: {y}, yaw: {yaw}')
        self.pub_nav_goal.publish(goal)
        return True



    def move_to_pick_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        pick_pose_msg = Pose()
        pick_pose_msg.position.x = x
        pick_pose_msg.position.y = y
        pick_pose_msg.position.z = z

        qua = quaternion_from_euler(roll, pitch, yaw)
        pick_pose_msg.orientation.x = qua[0]
        pick_pose_msg.orientation.y = qua[1]
        pick_pose_msg.orientation.z = qua[2]
        pick_pose_msg.orientation.w = qua[3]


        self.pub_arm_pick_pose.publish(pick_pose_msg)
        rospy.logwarn(f'pick pose msg has been published, pose: \n{pick_pose_msg.position}, \nquaternion: \n{pick_pose_msg.orientation}')
        return True
        


    def move_to_place_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        place_pose_msg = Pose()
        place_pose_msg.position.x = x
        place_pose_msg.position.y = y
        place_pose_msg.position.z = z

        qua = quaternion_from_euler(roll, pitch, yaw)
        place_pose_msg.orientation.x = qua[0]
        place_pose_msg.orientation.y = qua[1]
        place_pose_msg.orientation.z = qua[2]
        place_pose_msg.orientation.w = qua[3]


        self.pub_arm_place_pose.publish(place_pose_msg)
        rospy.logwarn(f'place pose msg has been published, pose: \n{place_pose_msg.position}, \nquaternion: \n{place_pose_msg.orientation}')
        return True