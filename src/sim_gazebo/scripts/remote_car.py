#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import keyboard

def get_state():
    keys = {'w': 0, 's': 0, 'a': 0, 'd': 0, 'q': 0, 'e': 0, 'p': 0}
    for i in keys:
        keys[i] = keyboard.is_pressed(i)
    x = keys['w'] - keys['s']
    y = keys['a'] - keys['d']
    z = keys['q'] - keys['e']
    other = keys['p']
    return x, y, z, other

class RobotInterface:
    def __init__(self, cmd_topic_name):
        self.pub_move_cmd = rospy.Publisher(cmd_topic_name, Twist, queue_size=1)
        rospy.on_shutdown(self.shutdown)
    
    def shutdown(self):
        print("shutting down the ros...")
    
    def move_cmd_send(self, move_cmd):
        self.pub_move_cmd.publish(move_cmd)
    
    def run(self):
        sb = get_state()
        cmd = Twist()
        cmd.linear.x = sb[0]
        cmd.linear.y = sb[1]
        cmd.angular.z = sb[2]
        self.move_cmd_send(cmd)
        return sb[3]

def main():
    rospy.init_node('remoteCar')
    small_car = RobotInterface('/sim/smallCar/cmd_vel')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        sta = small_car.run()
        rate.sleep()
        if sta:
            rospy.signal_shutdown('sb')

if __name__ == "__main__":
    main()
