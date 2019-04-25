#!/usr/bin/env python
import rospy
from curtsies import Input
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int32
def main():
    rospy.init_node('teleop', anonymous = True)
    rate = rospy.Rate(10)
    sendRate = rospy.Rate(1);
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size = 1)

    PI = 3.14159265897
    forward = Twist()
    forward.linear.x = 3

    backward = Twist()
    backward.linear.x = -1 * forward.linear.x

    turnRight = Twist()
    turnRight.angular.z = -PI / 2

    turnLeft = Twist()
    turnLeft.angular.z = PI / 2

    b = Bool()
    b.data = True

    i = Int32()

    with Input(keynames='curses') as input_generator:
        for e in input_generator:
            #print (e)
            if e == 'p':
                break
            elif e == 'KEY_UP':
                pub.publish(forward)
            elif e == 'KEY_DOWN':
                pub.publish(backward)
            elif e == 'KEY_RIGHT':
                pub.publish(turnRight)
            elif e == 'KEY_LEFT':
                pub.publish(turnLeft)


if __name__ == '__main__':
    main()
