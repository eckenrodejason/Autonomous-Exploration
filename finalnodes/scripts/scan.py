#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def subScan(data):
    maxRange = max(data.ranges)
    minRange = min(data.ranges)
    maxIndex = data.ranges.index(maxRange)
    minIndex = data.ranges.index(minRange)
    maxDegrees = maxIndex * 0.0065540750511 * 57.2957795
    minDegrees = minIndex * 0.0065540750511 * 57.2957795
    print('Max:' + str(maxRange) + ' towards ' + str(maxDegrees))
    print('Min:' + str(minRange) + ' towards ' + str(minDegrees))

def main():
    rospy.init_node('getscan', anonymous = True)
    rate = rospy.Rate(20)

    rospy.Subscriber('/scan', LaserScan, subScan)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
