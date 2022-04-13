import rospy
from geometry_msgs.msg import PoseStamped
import math


class PoseInterface:
    def __init__(self, topic_name, size, scale, diff):
        self.size = size
        self.pose = rospy.Subscriber(topic_name, PoseStamped, self.pose_callback, queue_size=1)
        self.scale = scale
        self.diff = diff
        self.x = 0
        self.y = 0
        self.yaw = 0

    def change_param(self, scale, diff):
        """
        :param scale: num
        :param diff: [v, >]
        :return:
        """
        self.scale += scale
        self.diff[0] += diff[0]
        self.diff[1] += diff[1]

    def quart_to_rpy(self, x, y, z, w):
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return roll, pitch, yaw

    def pose_callback(self, data):
        if self.scale < 1:
            self.scale = 1
        display_center_point = self.size[0]/2
        origin_center_point = 1024
        pixel_distance_param = 20 * self.scale

        x = data.pose.position.x
        y = data.pose.position.y
        x *= pixel_distance_param
        y *= pixel_distance_param
        x -= self.diff[1] * self.scale
        y -= self.diff[0] * self.scale
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

