#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::Pose goalToSee;

int coordToArr(int x, int y) {
  int val = std::abs(-10 - x) * 2 + std::abs(10 - y) * 2 * 40;
  return val;
}

int roundHalf(float a) {
  return (floor((a*2)+0.5)/2);
}

void getViewedMap(const std_msgs::String msg) {
  for (int i = 0; i < 1600; ++i) {
    if (msg.data[i] == '0') {
      float y = i / 40;
      float x = i % 40;
      goalToSee.position.x = -1 * 9 + (x * 0.5); 
      goalToSee.position.y = 9 - (y * 0.5);
      tf2::Quaternion q;
      q.setRPY(0,0,0);
      tf2::fromMsg(goalToSee.orientation, q);
      break;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calcGoal");
  ros::NodeHandle nh;

  ros::Rate rate(20);
  ros::Subscriber sub = nh.subscribe<std_msgs::String>("viewed_map", 1000, getViewedMap);
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("goal", 1); 

  while (ros::ok()) {
    ROS_INFO_STREAM(goalToSee);
    pub.publish(goalToSee);
    ros::spinOnce();
    rate.sleep();
  }
}
