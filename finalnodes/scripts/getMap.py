#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

def subGrid(data):
    print(len(data.data))

def main():
    rospy.init_node('getscan', anonymous = True)
    rate = rospy.Rate(20)

    rospy.Subscriber('/map', OccupancyGrid, subGrid)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
