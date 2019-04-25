#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <angles/angles.h>


//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*This is the target destination we are trying to get to also we are assuming that the user won't try to get to a point that has a collision*/
float dest_x;
float dest_y;
float dest_theta;

/*This is current pose*/
float curr_x;
float curr_y;
float curr_theta;

bool newValue = false;
float PI =  3.14159265358979323846;
int hertz = 20;

void currPose(const geometry_msgs::Pose msg){
    curr_x = msg.position.x;
    curr_y = msg.position.y;
    curr_theta = angles::normalize_angle_positive(tf2::getYaw(msg.orientation));

 	// ROS_INFO_STREAM("curr x "<< curr_x << " curr y "<< curr_y << " curr Theta " << curr_theta);   

}

void destPose(const geometry_msgs::Pose msg){
    dest_x = msg.position.x;
    dest_y = msg.position.y;
    dest_theta = angles::normalize_angle_positive(tf2::getYaw(msg.orientation));

	//ROS_INFO_STREAM("Target x "<< dest_x << " Target y "<< dest_y << " target Theta " << dest_theta);

	newValue = true;

}

float absolute(float num){
	return (num >= 0) ? num : num * -1;
}

/*gives me the angle I need to orient myself to move to (NOT RELATIVE TO MYSELF, JUST THE ANGLE I NEED!)*/
float targetAngle(){

	float dist_x = dest_x - (curr_x);
	float dist_y = dest_y - (curr_y);

	float destAngle = atan(absolute(dist_y/dist_x));

	if(dist_x < 0 && dist_y < 0){
		destAngle += PI;
	}			
	else if(dist_x > 0 && dist_y < 0){
		destAngle = 2*PI - destAngle;
	}
	else if(dist_x < 0 && dist_y > 0){
		destAngle = PI - destAngle;
	}

	//convert to -pi - pi range
	//return (destAngle <= PI) ? destAngle : destAngle - 2 * PI;

	return destAngle;
} 

float dist(){
	return sqrt(pow(dest_x - curr_x, 2) + pow(dest_y - curr_y, 2));

}


/*Takes in pose and sends it to topic*/
int main(int argc, char** argv){ 

	ros::init(argc, argv, "gotopose");
	ros::NodeHandle nh;
	ros::Rate rate(hertz);

	ros::Subscriber subDest = nh.subscribe("targetpose", 1000, destPose);
	ros::Subscriber subCurr = nh.subscribe("perfect_localization", 1000, currPose);
	
	//ros::Subscriber subTF = nh.subscribe("/tf2_ros", 1000, &tf2MessageReceived);

	ros::Publisher pubTwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
	ros::Publisher pubRestart = nh.advertise<std_msgs::Empty>("restartTopic", 1000);


	for (int i = 0; i < 80; ++i) {
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::Twist toRotate; 

	geometry_msgs::Twist toMove;

	geometry_msgs::Twist stop;


	ROS_INFO_STREAM("Waiting for a Position..");
	while(ros::ok()){//this is a constant infinite loop

		if(newValue){//if user inputted a new value, move to that place
			ROS_INFO_STREAM("I got a new destination");
			ros::spinOnce();

			float distance = dist();

			float currAngle = curr_theta;

			float destAngle = targetAngle();	

			ROS_INFO_STREAM("Target x "<< dest_x << " Target y "<< dest_y << " target Theta " << dest_theta);

			// ROS_INFO_STREAM("Angle I need = "<<destAngle <<" current Angle "<< currAngle);

			// float targetRotation = destAngle - currAngle; //may need to change if angle between rotation is really tiny
			// /*So now we have the distance we need to traverse and how much we need to rotate*/

			// ROS_INFO_STREAM("Distance to target "<< distance << " | Rotation I need to take "<< targetRotation);


			//this is where my calibration and first rotation happens

rerotate:			
			float calibration = targetAngle() - curr_theta;
			while(!(calibration < 0.1 && calibration > -0.1)){ 
			//	ROS_INFO_STREAM("Target x "<< dest_x << " Target y "<< dest_y << " target Theta " << dest_theta);

				toRotate.angular.z = calibration * 2; 
				toRotate.linear.x = 0;
				pubTwist.publish(toRotate);
				ros::spinOnce();
				rate.sleep();

				calibration = targetAngle() - curr_theta;
				ROS_INFO_STREAM("rotating only: angle = " << calibration);

			//	ROS_INFO_STREAM ("dest "<<destAngle <<" curr Angle "<<tf::getYaw(transformStamped.transform.rotation) << " calibration "  << calibration);

			}

			pubTwist.publish(stop);
			rate.sleep();

			//ROS_INFO_STREAM("Finished first rotation");

			//The actual moving which controls both linear and angular, deaccalarates togiven bound
			while(distance > .1){ //Degree of accuracy
				//transformStamped = buffer.lookupTransform("odom", "base_link",ros::Time(0));

				if (absolute(calibration) > .3){ goto rerotate; }
				toMove.angular.z = calibration; 
				toMove.linear.x = distance;
				pubTwist.publish(toMove);
				ros::spinOnce();
				rate.sleep();

				distance = dist();
				calibration = targetAngle() - curr_theta; //our calibration is based on differenct angles because we're moving now

				ROS_INFO_STREAM("moving:  angle  = "<< calibration << " | distance = " << distance) ;
		

			}

			pubTwist.publish(stop);
			rate.sleep();

			ROS_INFO_STREAM("Finished.");

			//tells getpose that this part is done
			std_msgs::Empty emMsg;
			for(int i = 0; i < hertz+2; i++){
				pubRestart.publish(emMsg);
				rate.sleep();
			}


			newValue = false;
		}
		ros::spinOnce();
		rate.sleep();

	}

}