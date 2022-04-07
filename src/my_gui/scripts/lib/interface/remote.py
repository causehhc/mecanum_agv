import rospy
from geometry_msgs.msg import Twist


class RemoteInterface:
    def __init__(self, topic_name):
        self.pub_move_cmd = rospy.Publisher(topic_name, Twist, queue_size=1)

    def move_cmd_send(self, move_cmd):
        cmd = Twist()
        cmd.linear.x = move_cmd[0]
        cmd.linear.y = move_cmd[1]
        cmd.angular.z = move_cmd[2]
        self.pub_move_cmd.publish(cmd)
        return move_cmd[3]


def main():
    rospy.init_node('hahaha')
    small_car = RemoteInterface('/sim/smallCar/cmd_vel')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ret = small_car.move_cmd_send([0, 0, 0, 0])
        rate.sleep()
        if ret:
            rospy.signal_shutdown('sb')


if __name__ == "__main__":
    main()
