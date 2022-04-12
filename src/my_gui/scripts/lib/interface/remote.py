import rospy
import keyboard
from geometry_msgs.msg import Twist


def get_state():
    keys = {'w': 0, 's': 0, 'a': 0, 'd': 0, 'q': 0, 'e': 0, 'p': 0}
    for i in keys:
        keys[i] = keyboard.is_pressed(i)
    x = keys['w'] - keys['s']
    y = keys['a'] - keys['d']
    z = keys['q'] - keys['e']
    other = keys['p']
    return x, y, z, other


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

    def run_once(self):
        status = get_state()
        self.move_cmd_send(status)

    def run(self, haha=1):
        rate = rospy.Rate(50)
        while True:
            self.run_once()
            rate.sleep()




def main():
    rospy.init_node('hahaha')
    small_car = RemoteInterface('/cmd_vel')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ret = small_car.move_cmd_send([0, 0, 0, 0])
        rate.sleep()
        if ret:
            rospy.signal_shutdown('sb')


if __name__ == "__main__":
    main()
