#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>


float timeToWait = 5.0;
int numberFailed = 0;

//jason edit
void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
    // ROS_INFO_STREAM("Service still running");
    // ROS_INFO_STREAM("Current pose (x,y) " <<
    //     fb->base_position.pose.position.x << "," <<
    //     fb->base_position.pose.position.y);
}

void fixamcl(ros::Publisher* pub, ros::ServiceClient* reset) {
  std_srvs::Empty emptymsg;
  reset->call(emptymsg);

  ros::Rate rate(20);

  geometry_msgs::Twist twist;
  twist.linear.x = 0.5;
  twist.angular.z = 0.2;
  for (int i = 0; i < 400; ++i) {
    pub->publish(twist);
    twist.linear.x = twist.linear.x + 0.05;
    twist.linear.x = twist.angular.z + 0.05;
    rate.sleep();
  }
}

void serviceActivated() {
    ROS_INFO_STREAM("Service received goal");
}

void serviceDone(const actionlib::SimpleClientGoalState& state,
     const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO_STREAM("Service completed");
    ROS_INFO_STREAM("Final state " << state.toString().c_str());
    //ros::shutdown();
}

// end Jason edit

bool mySendGoal(float x, float y, ros::Publisher* pub,
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac,
   geometry_msgs::Twist* twist,
   ros::ServiceClient* reset) {

  ros::Rate rate(20);

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1;

  ROS_WARN_STREAM("Goal: " << goal.target_pose.pose.position.x << " , " << goal.target_pose.pose.position.y);
  ros::Time start = ros::Time::now();
  ac->sendGoal(goal,&serviceDone,&serviceActivated, &serviceFeedback);
  while (ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED
    && ros::Time::now() - start < ros::Duration(timeToWait)) {
    }

  if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Success");
    timeToWait = timeToWait - 0.5;
    return true;
  }
  else {
    ROS_ERROR_STREAM("Failure, going back 1 meter");
    //sometimes you might get really close to goal but then it took too long to get there
    //go back, but give the next round some more time to get there. when you DO get to the goal,
    //expect less time to get to the next.
    ac->cancelGoal();
    for (int i = 0; i < 60; ++i) {
      pub->publish(*twist);
      rate.sleep();
    }
    timeToWait = timeToWait + 3.5;
    numberFailed++;
    //if you fail more than 3 times you should reset the pose.
    if (numberFailed > 3) {
      numberFailed = 0;
      fixamcl(pub, reset);
    }
    return false;
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "movebasetest");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("/global_localization");
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  while(!ac.waitForServer()) {
  }
  ROS_INFO_STREAM("Server Available!");

  geometry_msgs::Quaternion q;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -8.0;
  goal.target_pose.pose.position.y = 8.0;
  goal.target_pose.pose.orientation.w = 1;

  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Success");
  else
    ROS_ERROR_STREAM("Failure");

  geometry_msgs::Twist twist;
  twist.linear.x = -0.75;

  bool toPositive = true;

  for (float y = 8.0; y > -8.5; y = y - 1.0) {
    if (toPositive) {
      for (float x = -8.0; x < 8.5; x = x + 1.0) {
        if (!mySendGoal(x, y, &pub, &ac, &twist, &reset)) { // if fails, then try again
          x = x - 0.5; //try diff point in case you tried going in to a wall
        }
      }
      toPositive = false;
    }
    else {
      for (float x = 8.0; x > -8.5; x = x - 1.0) {
        if (!mySendGoal(x, y, &pub, &ac, &twist, &reset)) { //if fails, then try again
          x = x + 0.5; //try different point, in case you tried going into a wall
        }
      }
      toPositive = true;
    }
  }

}
