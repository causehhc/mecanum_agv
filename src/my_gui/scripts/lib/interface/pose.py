import rospy
from geometry_msgs.msg import PoseStamped
import math


class PoseInterface:
    def __init__(self, topic_name):
        self.pose = rospy.Subscriber(topic_name, PoseStamped, self.pose_callback, queue_size=1)
        self.SCALE = 5
        self.x = 0
        self.y = 0
        self.yaw = 0

    def quart_to_rpy(self, x, y, z, w):
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return roll, pitch, yaw

    def pose_callback(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        display_center_point = 300
        origin_center_point = 1024
        pixel_distance_param = 20 * self.SCALE

        x *= pixel_distance_param
        y *= pixel_distance_param
        x += origin_center_point
        y += origin_center_point
        x /= (origin_center_point / display_center_point)
        y /= (origin_center_point / display_center_point)

        self.x = int(x)
        self.y = int(y)
        _, _, self.yaw = self.quart_to_rpy(
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )

