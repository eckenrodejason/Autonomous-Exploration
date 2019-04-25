#include <ros/ros.h>
#include <tf2/utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <angles/angles.h>
#include <map>
#include <std_msgs/String.h>

std::string graph = "";
// std_msgs::Int32MultiArray arr;
geometry_msgs::Pose current_pose;
bool gotMap = false;
bool finished = false;
nav_msgs::OccupancyGrid mainMap;
int square = 10; // square x square

float PI =  3.14159265358979323846;

int matToArr(int row, int col, int row_tar, int col_tar){
  return (row_tar-1) * col + col_tar - 1;
}

int coordToArr(int x, int y) {
  int val = std::abs(-10 - x) * 2 + std::abs(10 - y) * 2 * 40;
  // std::cout << "returning " << val << "\n";
  return val;
}

float roundForty(int a) {
  return (floor((a / 40) + 40) * 40);
}

void markRead(int index) {
  roundForty(index);
  if (graph[index] != '1')
    graph[index] = '2';
  if (graph[index + 40] != '1')
    graph[index + 40] = '2';
  if (graph[index - 40] != '1')
    graph[index - 40] = '2';
  if (graph[index + 41] != '1')
    graph[index + 41] = '2';
  if (graph[index - 41] != '1')
    graph[index - 41] = '2';
  if (graph[index - 1] != '1')
    graph[index - 1] = '2';
  if (graph[index + 1] != '1')
    graph[index + 1] = '2';
}

int roundHalf(float a) {
  return (floor((a*2)+0.5)/2);
}


bool occupied (int i, int j){
  for(int l = 0; l < square; l++) {
    for(int k = 0; k < square; k++) {
      if((int)mainMap.data[matToArr(800,800,j+l,i+k)] != 0) {
         return true;
      }
    }
  }
  return false;
}

void printGraph() {
  std::cout << "\n\n";
  for(int i = 0; i < graph.length(); i++){
    std::cout << graph[i];
    if((i+1)%(400/square) == 0){
      std::cout<< '\n';
    }
  }

}

void discretize(const nav_msgs::OccupancyGrid msg){
  if (!gotMap) {
    mainMap = msg;
    gotMap = true;
  //gets me to the start of every square
    for (int j = 200; j < 600; j+=square) {
      for (int i = 200; i < 600;i+=square) {
        graph += (occupied(i,j)) ? '1' : '0';
      }
    }

/*flips y coordinates graph because we need to  why? ¯\_(ツ)_/¯*/
    std::string temp = "";
    for(int j = 40; j >= 1; j--){
      for(int i = 1; i <= 40; i++){
        temp += graph[matToArr(40,40,j,i)];
      }
    }
    graph = temp;
    printGraph();

    finished = true;
 }
}



void subPose(const geometry_msgs::Pose msg) {
  current_pose = msg;
  float angle = tf2::getYaw(msg.orientation);
  angle = angles::normalize_angle_positive(angle);
  float seeUpToX = roundHalf(2 * cos(angle) + current_pose.position.x);
  float seeUpToY = roundHalf(2 * sin(angle) + current_pose.position.y);

  float rounded_current_X = roundHalf(current_pose.position.x);
  float rounded_current_Y = roundHalf(current_pose.position.y);

  // std::cout << "I can see from " << rounded_current_X << " up to " << roundHalf(seeUpToX)
    // << " and " << rounded_current_Y << " to " << roundHalf(seeUpToY) << "\n";


  float iterateX = 0.5;
  float iterateY = 0.5;

  float rounded_current_X_tmp = rounded_current_X;
  float rounded_current_Y_tmp = rounded_current_Y;

  // std::cout << angle << " is greater than " << PI << " but is less than " << (PI * 3 / 2) << "\n";

  if(angle < PI/2){
    for (rounded_current_X = rounded_current_X_tmp; rounded_current_X <= seeUpToX; rounded_current_X += iterateX) {
      for (rounded_current_Y = rounded_current_Y_tmp; rounded_current_Y <= seeUpToY; rounded_current_Y += iterateY) {
        int index = coordToArr(rounded_current_X, rounded_current_Y);
        markRead(index);
      }
    }
  }
  else if(angle >= PI/2 && angle < PI){
    for (rounded_current_X = rounded_current_X_tmp; rounded_current_X >= seeUpToX; rounded_current_X -= iterateX) {
      for (rounded_current_Y = rounded_current_Y_tmp; rounded_current_Y <= seeUpToY; rounded_current_Y += iterateY) {
        int index = coordToArr(rounded_current_X, rounded_current_Y);
        markRead(index);
      }
    } 
  }
  else if(angle >= PI && angle < (PI * 3 / 2)){
    for (rounded_current_X = rounded_current_X_tmp; rounded_current_X >= seeUpToX; rounded_current_X -= iterateX) {
      for (rounded_current_Y = rounded_current_Y_tmp; rounded_current_Y >= seeUpToY; rounded_current_Y -= iterateY) {
        int index = coordToArr(rounded_current_X, rounded_current_Y);
        markRead(index);
      }
    }
  }
  else {
    for (rounded_current_X = rounded_current_X_tmp; rounded_current_X <= seeUpToX; rounded_current_X += iterateX) {
      for (rounded_current_Y = rounded_current_Y_tmp; rounded_current_Y >= seeUpToY; rounded_current_Y -= iterateY) {
        int index = coordToArr(rounded_current_X, rounded_current_Y);
        markRead(index);
      }
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "discreteMap");
  ros::NodeHandle nh;
  ros::Rate rate(20);
  ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, discretize);
  ros::Subscriber sub2 = nh.subscribe<geometry_msgs::Pose>("perfect_localization", 1000, subPose);
  ros::Publisher pub = nh.advertise<std_msgs::String>("viewed_map", 1000);


  while (ros::ok()) {
    std_msgs::String msg;
    msg.data = graph;
    pub.publish(msg);
    finished = false;

    printGraph();
    ros::spinOnce();
    rate.sleep();
  }

}
