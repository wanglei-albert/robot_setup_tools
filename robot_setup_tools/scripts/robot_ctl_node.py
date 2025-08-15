import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from yunhai_msgs.srv import SendGoal, SendGoalRequest
from franka_msgs.msg import FrankaState
from franka_msgs.srv import GetTFEcho, GetTFEchoRequest, GetTFEchoResponse
from franka_msgs.srv import SetGoalwithCartesian, SetGoalwithCartesianRequest

class RobotCtrlNode():
    def __init__(self):
        rospy.loginfo('ArmCtrlNode has been started!')
        self._nav_goal_service_name = '/yunhai_go_pose'
        self._move_arm_cart_service_name = 'franka_cartesianGoal'
        self._echo_tf_service_name = '/tf_echo_server'
        self._nav_to_goal_cli = rospy.ServiceProxy(self._nav_goal_service_name, SendGoal)
        self._get_cur_arm_pose_cli = rospy.ServiceProxy(self._echo_tf_service_name, GetTFEcho)
        self._move_arm_to_cart_pose_cli = rospy.ServiceProxy(self._move_arm_cart_service_name, SetGoalwithCartesian)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.on_arm_pose_msg)
        self._arm_pose = FrankaState()


    def on_arm_pose_msg(self, msg: FrankaState):
        self._arm_pose = msg

    def get_cur_arm_pose(self):
        try:
            rospy.wait_for_service(self._echo_tf_service_name, 5)
            req = GetTFEchoRequest()
            req.parent_link = 'pand_link0'
            req.child_link = 'pand_link8'

            resp = self._get_cur_arm_pose_cli(req)

            return True, {'x': resp.translation.x, 'y': resp.translation.x, 'z': resp.translation.z,
                          'rx': resp.rpy.x, 'ry': resp.rpy.y, 'rz': resp.rpy.z}
        except Exception as e:
            err_str = f'failed to get current arm psoe, exception: {e}'
            rospy.logerr(err_str)
            return False, err_str


    def nav_to_goal(self, x:float, y:float, yaw: float, use_coord_nav:bool, marker=''):
        try:
            rospy.wait_for_service(self._nav_goal_service_name, 5)

            rospy.loginfo(f'navigation goal has been published, goal: x: {x}, y: {y}, yaw: {yaw}')
            nav_req = SendGoalRequest()
            nav_req.marker = marker
            nav_req.x, nav_req.y, nav_req.theta = x, y, yaw
            nav_req.use_location = use_coord_nav
            
            resp = self.self._nav_to_goal_cli(nav_req)

            return True, resp.result
        except Exception as e:
            rospy.logerr(f'failed to navigating to goal pose(x, y, yaw): {x}, {y}, {yaw}')
            return False, f'exception: {e}'



    def move_to_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        try:
            motion_req = SetGoalwithCartesianRequest()
            motion_req.x = x
            motion_req.y = y
            motion_req.z = z
            motion_req.rx = roll
            motion_req.ry = pitch
            motion_req.rz = yaw
            motion_req.gripper_width = 0.0
            motion_req.gripper_speed = 0.0
            motion_req.gripper_force = 0.0
            motion_req.mode = 0

            resp = self._move_arm_to_cart_pose_cli(motion_req)

            rospy.logwarn(f'pick pose msg has been published, pose: {x}, {y}, {z}, rotation: {roll}, {pitch}, {yaw}')
            return resp.result, resp.message
        except Exception as e:
            rospy.logerr(f'failed to navigating to goal pose(x, y, yaw): {x}, {y}, {yaw}')
            return False, f'exception: {e}'