#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

current_pose = Pose()

def subState(data):
    global current_pose
    index = data.name.index("/")
    current_pose = data.pose[index]

def main():
    rospy.init_node('getStates', anonymous = True)
    rate = rospy.Rate(20)

    rospy.Subscriber('/gazebo/model_states', ModelStates, subState)
    pub = rospy.Publisher('perfect_localization', Pose, queue_size = 1)

    while not rospy.is_shutdown():
        print(current_pose)
        pub.publish(current_pose)
        rate.sleep()


if __name__ == '__main__':
    main()
