
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdlib.h> 
#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/MotorPower.h>


class LineFollower
{
	public:
		LineFollower(ros::NodeHandle &n);
		void stateMachine();
		void updateCliff(const kobuki_msgs::CliffEventConstPtr msg);
		void updateBumper(const kobuki_msgs::BumperEventConstPtr msg);
		void updateRobotSensors(const kobuki_msgs::SensorStateConstPtr msg);
		bool isRunning();
	private:
		ros::NodeHandle node;
		ros::Subscriber cliff_sub , bumper_sub , robot_sub;
		ros::Publisher drive_pub;
		geometry_msgs::Twist velo_message;
		int lower_color_threshold , higher_color_threshold;
		int previous_state ,current_state;
		int rl , rh , ll , lh;
		int samples , rs , ls;
		bool keepRunning;
		std::ofstream sensorLog;
		int16_t previous_center_val;

};

LineFollower::LineFollower(ros::NodeHandle &n):
	lower_color_threshold(0),
	higher_color_threshold(100),
	previous_state(0),
	rl(1550),
	rh(1550),
	ll(1550),
	lh(1550),
	current_state(5)
{
	keepRunning = true;
	node = n;
	sensorLog.open("sensorLog.txt");
	drive_pub = node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);

	bumper_sub = node.subscribe("mobile_base/events/bumper", 10, &LineFollower::updateBumper, this);
	//cliff_sub = node.subscribe("mobile_base/events/cliff" , 10 , &LineFollower::updateCliff , this); 
	robot_sub = node.subscribe("mobile_base/sensors/core" , 10 , &LineFollower::updateRobotSensors , this); 

	ROS_INFO_STREAM("Init Line Follower\n");
}

void LineFollower::stateMachine()
{
	int new_state = 1;
	if(previous_state != new_state)
	{
		ROS_INFO_STREAM("Old state: " << previous_state << " New State: " << new_state << "\n");
	}
	//previous_state = current_state;
	//current_state = new_state;
}

void LineFollower::updateCliff(const kobuki_msgs::CliffEventConstPtr msg)
{
	ROS_INFO_STREAM("START PACKET \n");
	ROS_INFO_STREAM(msg->sensor << " " << msg->state << " " << msg->bottom << "\n");
}

void LineFollower::updateBumper(const kobuki_msgs::BumperEventConstPtr msg)
{
	//ROS_INFO_STREAM(msg->state);
	//Quit Program
	keepRunning = false;
	velo_message.linear.x = 0.0;
	velo_message.linear.y = 0.0;
	velo_message.linear.z = 0.0;
	velo_message.angular.x = 0.0;
	velo_message.angular.y = 0.0;
	velo_message.angular.z = 0.0;
	drive_pub.publish(velo_message);
	ROS_INFO_STREAM("RL: " << rl << " RH: " << rh << " LL: " << ll << " LH: " << lh);
	sensorLog.close();
}

bool LineFollower::isRunning()
{
	return(keepRunning);
}

void LineFollower::updateRobotSensors(const kobuki_msgs::SensorStateConstPtr msg)
{
	/*
	Motion States:
		Turn Right
			If right val is triggered
		Turn Left
			If left val is triggered
		Keep Straight
			Else
	*/
	int16_t curr_right_val = msg->bottom[0];
	int16_t curr_center_val = msg->bottom[1];
	int16_t curr_left_val = msg->bottom[2];

	samples += 1;
	rs += curr_right_val;
	ls += curr_left_val;

	if(rl > curr_right_val){
		rl = curr_right_val;
	}
	
	if(rh < curr_right_val){
		rh = curr_right_val;
	}
	
	if(ll > curr_left_val){
		ll = curr_left_val;
	}
	
	if(lh < curr_left_val){
		lh = curr_left_val;
	}
	sensorLog << curr_right_val << "," << curr_left_val << "," << curr_center_val <<  "\n";
	//ROS_INFO_STREAM("R: " << curr_right_val << " L: " << curr_left_val << "\n");
	//ROS_INFO_STREAM(rs/samples);
	//ROS_INFO_STREAM(ls/samples);
	//1492 RH: 1593 LL: 1437 LH: 1631

	if(curr_right_val > 1600)
	{
		if(current_state != 0)
		{
			ROS_INFO_STREAM("Turn Right\n");
			current_state = 0;
			velo_message.linear.x = 0.0;
			velo_message.linear.y = 0.0;
			velo_message.linear.z = 0.0;
			velo_message.angular.x = 0.0;
			velo_message.angular.y = 0.0;
			velo_message.angular.z = -2.0;
		}
	}
	else if(curr_left_val > 1600)
	{
		if(current_state != 1)
		{
			ROS_INFO_STREAM("Turn Left\n");	
			current_state = 1;
			velo_message.linear.x = 0.0;
			velo_message.linear.y = 0.0;
			velo_message.linear.z = 0.0;
			velo_message.angular.x = 0.0;
			velo_message.angular.y = 0.0;
			velo_message.angular.z = 2.0;
		}
	}
	else
	{
		if(current_state != 2)
		{
			ROS_INFO_STREAM("Keep Straight\n");
			current_state = 2;	
			velo_message.linear.x = 0.2;
			velo_message.linear.y = 0.0;
			velo_message.linear.z = 0.0;
			velo_message.angular.x = 0.0;
			velo_message.angular.y = 0.0;
			velo_message.angular.z = 0.0;
		}	
	}

	drive_pub.publish(velo_message);
	


	/*
	if(previous_center_val > curr_val + 50 || previous_center_val < curr_val - 50)
	{
		ROS_INFO_STREAM(curr_val);
		ROS_INFO_STREAM("\n");		
		previous_center_val = curr_val;
	}
	*/
}


int main(int argc , char** argv)
{
	ros::init(argc , argv , "line_following");
	ros::NodeHandle n;
	
	LineFollower follower(n);
	 
	while(ros::ok() && follower.isRunning()){
		ros::spinOnce();
	}
	
	return 0;
}