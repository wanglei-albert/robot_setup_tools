import rospy
from aruco_msgs.msg import MarkerArray

class ArucoReadNode():
    def __init__(self):
        rospy.loginfo('ArmCtrlNode has been started!')

    def read_markers(self, marker_ids = None):
        try:
            rospy.loginfo(f'reading markers....')
            aruco_msg = rospy.wait_for_message('markers', MarkerArray, 5)
            result = []
            for marker in aruco_msg.markers:
                marker_info = {}
                marker_info['id'] = marker.id
                marker_info['confidence'] = marker.confidence
                marker_info['pose'] = {
                    'x': marker.pose.pose.position.x,
                    'y': marker.pose.pose.position.y,
                    'z': marker.pose.pose.position.z,
                    'qx': marker.pose.pose.orientation.x,
                    'qy': marker.pose.pose.orientation.y,
                    'qz': marker.pose.pose.orientation.z,
                    'qw': marker.pose.pose.orientation.w,
                }

                result.append(marker_info)

            return True, result
        except Exception as e:
            rospy.logerr(f'failed to read markers, exception: {e}')
            return False, f'exception: {e}'
        