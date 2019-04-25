#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <map>
//#include <tf/Matrix3x3.h>


using namespace std;

geometry_msgs::Pose2D currentPose;
float PI =  3.14159265358979323846;

struct treasureChest{

  map<string, pair<float, float> > chest;

  treasureChest(){}

  void printTreasures(){
    for(map<string, pair<float, float> >::iterator it = chest.begin(); it !=chest.end(); it++){
      ROS_INFO_STREAM(it->first<< " location x: " << it->second.first << " y: "<< it->second.second );
    }

  }

  void addTreasure(string id, float x, float y){
    if(chest.count(id)){
      // ROS_INFO_STREAM("We already got that treasure my dude");

    }
    else{

      if(currentPose.theta < PI/2){
        chest.insert(make_pair(id, make_pair (currentPose.x+x, currentPose.y+y)));
      }
      else if(currentPose.theta >= PI/2 && currentPose.theta< PI){
        chest.insert(make_pair(id, make_pair (currentPose.x-x, currentPose.y+y)));
      }
      else if(currentPose.theta >= PI && currentPose.theta< PI * 3/2){
        chest.insert(make_pair(id, make_pair (currentPose.x-x, currentPose.y-y)));
      }
      else {
        chest.insert(make_pair(id, make_pair (currentPose.x+x, currentPose.y-y)));
      }

      printTreasures();

    }
  }

};

treasureChest tc;

void getPose(const geometry_msgs::Pose msg){
  float amclAngle = tf::getYaw(msg.orientation);
  currentPose.x = msg.position.x;
  currentPose.y = msg.position.y;

  geometry_msgs::Quaternion q;
  q = msg.orientation;
  currentPose.theta = angles::normalize_angle_positive(tf::getYaw(q));

}

void sawTreasure(const logical_camera_plugin::logicalImage msg) {

  tc.addTreasure(msg.modelName ,msg.pose_pos_x, msg.pose_pos_y);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "detectObjects");
  ros::NodeHandle nh;

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<logical_camera_plugin::logicalImage>("/objectsDetected", 1000, sawTreasure);
  ros::Subscriber subPose = nh.subscribe("perfect_localization", 1000, getPose);


  while (ros::ok()) {
    ros::spinOnce();
  }

}
