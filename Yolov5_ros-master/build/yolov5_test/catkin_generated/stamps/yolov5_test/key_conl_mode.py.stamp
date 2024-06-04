#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import sys
import tty
import termios

# 封装的键盘读取函数
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd, termios.TCSAFLUSH)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, old_settings)
    return ch


if __name__ == '__main__':
    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('Key_mode', Int32, queue_size=10)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        key = getch()
        if key == 'A':
            msg = 1
        elif key == 'B':
            msg = 2
        pub.publish(msg)
        rate.sleep()

