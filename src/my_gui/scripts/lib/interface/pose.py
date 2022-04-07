import rospy
from geometry_msgs.msg import PoseStamped


class PoseInterface:
    def __init__(self, topic_name):
        self.pose = rospy.Subscriber(topic_name, PoseStamped, self.pose_callback, queue_size=1)
        self.SCALE = 5
        self.x = 0
        self.y = 0

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

        self.x = x
        self.y = y

