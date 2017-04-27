#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sstream>
#include <math.h>  
#include <iostream>
#include <fstream>
#include <stdlib.h> 
#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/MotorPower.h>
#include <vector>

#include "rapidjson/document.h"

/*
	WayPoint Class
*/

class WayPoint{
	public:
		WayPoint(double xPoint, double yPoint);
		WayPoint();
		~WayPoint();
		double getX();
		double getY();
		double X;
		double Y;
};

WayPoint::WayPoint(double xPoint , double yPoint): X(xPoint), Y(yPoint){
}

WayPoint::WayPoint(){
	X = 0;
	Y = 0;
}

WayPoint::~WayPoint(){
	//Add deletion items here if needed
}

double WayPoint::getX(){
	return(X);
}

double WayPoint::getY(){
	return(Y);
}

/*
	Path Executor Class
*/

class PathExecutor{
	public:
		PathExecutor(ros::NodeHandle &n, std::vector<WayPoint> wps);
		void updatePose(const nav_msgs::Odometry msg);
		void updateBumper(const kobuki_msgs::BumperEventConstPtr msg);
		void updatePath(const std_msgs::String::ConstPtr& msg);
		void stateMachine();
		void resetRobot();
		void tester();
		int getRobotIP();
		bool isRunning();
		WayPoint getLookAheadPoint(WayPoint currentLocation);
	private:
		ros::NodeHandle node;
		ros::Subscriber pose_sub, bumper_sub , path_sub;
		ros::Publisher drive_pub;
		std::vector<WayPoint> pointVector;
		geometry_msgs::Pose pose_message;
		geometry_msgs::Twist velo_message;
		WayPoint previousSetPoint;
		WayPoint currentSetPoint;
		std::string ipAddress;
		int state;
		double error;
		double rawZ;
		double currentAngle;
		double previousAngle;
		double setPointAngle;
		double startAngle;
		double xPred;
		double yPred;
		bool done;
		bool newDataRecieved;
		double xStart;
		double yStart;
		bool justStarted;
		WayPoint current;
		WayPoint goalPoint;
		double goalPointRX;
		double goalPointRY;
		const double PI  = 3.141592653589793238463;
		const double lookAhead = 0.25;
		const double width = 0.23;
		std::ofstream pathLog;

};

PathExecutor::PathExecutor(ros::NodeHandle &n , std::vector<WayPoint> wps):pointVector(wps){
	node = n;
	state = 0;
	resetRobot();
	pose_sub = node.subscribe("odom", 10, &PathExecutor::updatePose, this);
	bumper_sub = node.subscribe("mobile_base/events/bumper", 10, &PathExecutor::updateBumper, this);
	path_sub = node.subscribe("robot_paths" , 10 , &PathExecutor::updatePath , this);
	drive_pub = node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);
	pathLog.open("pathLog.txt");
	pathLog << "Robot X,Robot Y,Robot Angle,Close X,Close Y,Goal X,Goal Y\n";
	ROS_INFO_STREAM("Init Path Executor\n");
}

void PathExecutor::resetRobot(){
	pointVector.clear();
	done = false;
	previousAngle = 0;
	startAngle = 100;
	xPred = 0;
	yPred = 0;
	justStarted = true;
	newDataRecieved = false;
	currentSetPoint = WayPoint(0,0);
	previousSetPoint = WayPoint(0,0);
}

void PathExecutor::stateMachine(){
	/*
		Case 0 - Reset
		Case 1 - Wait for data then parse
		Case 2 - Get new point
		Case 3 - Execute
		Case 4 - Finish
	*/
	double gamma;
	double lookAheadDistance;
	double lookAheadAngle;
	double radius;
	double chord;
	double crossProduct;
	double crossTerm;
	double my;
	double mx;
	double dx;
	double dy;
	double speed;
	switch(state){
		case 0:
			//Reset State
			ROS_INFO_STREAM("State 0");
			getRobotIP(); //Reset IP address
			resetRobot();
			state = 1;
			break;
		case 1:
			//Wait for Data
			if(newDataRecieved){
				state = 2;
			}
			//ROS_INFO_STREAM("Waiting for data.");
			//ROS_INFO_STREAM(ipAddress);
			//tester();
			//done = true;
			break;
		case 2:
			newDataRecieved = false;
			//Get new lookahead point
			//ROS_INFO_STREAM("State 1");
			current = WayPoint(xPred , yPred);
			goalPoint = getLookAheadPoint(current);
			goalPointRX = goalPoint.getX() - current.getX() ;//(goalPoint.getX() - current.getX())*std::cos(currentAngle) + (goalPoint.getY() - current.getY())*std::sin(currentAngle); //Relative to robot
			goalPointRY = goalPoint.getY() - current.getY();//-(goalPoint.getX() - current.getX())*std::sin(currentAngle) + (goalPoint.getY() - current.getY())*std::cos(currentAngle); //Relative to robot
			state = 3;
			break;
		case 3:
			//ROS_INFO_STREAM("State 2");
			/*
			gamma = -(2 * goalPointRX)/(std::pow(lookAhead , 2)); //-(2 * goalPointRX)/(std::pow(goalPointRY , 2)); 
			chord = std::sqrt(std::pow(goalPointRX , 2) + std::pow(goalPointRY , 2));
			radius = (chord/2)/std::sin(gamma/2);
			*/
			dx = current.getX() - goalPoint.getX();
			dy = current.getY() - goalPoint.getY();
			crossProduct = goalPointRX * std::sin(currentAngle) - goalPointRY * std::cos(currentAngle);
			my = (crossProduct > 0 ? -1 : 1) * std::cos(currentAngle);
			mx = (crossProduct > 0 ? 1 : -1) * std::sin(currentAngle);
			crossTerm = mx * dx + my * dy;
			gamma = 0;
			speed = 0.2;
			if(std::abs(crossProduct) > 1e-5 && std::abs(crossTerm) > 1e-5){
				radius = 0.5 * std::abs(dx * dx + dy * dy)/crossTerm;
				/*
				gamma = -0.3/radius;
				if (crossProduct > 0){
					gamma = 0.3/radius;
				}
				*/
				gamma = speed / radius;
				if(gamma > 10){
					gamma = 10;
					speed = gamma * radius;
				}

				if(crossProduct > 0){
					gamma = -gamma;
				}
				
			}
			state = 2;
			ROS_INFO_STREAM("CURRENT POINT(" << xPred << "," << yPred << ") GOAL POINT (" << goalPoint.getX() << "," << goalPoint.getY() << ") GAMMA " << gamma << " RADIUS " << radius);
			velo_message.linear.x = speed;
			velo_message.angular.z = gamma;
			break;
		case 4:
			ROS_INFO_STREAM("State 4");
			//Move Forward State
			break;
	}
	drive_pub.publish(velo_message);
}

void PathExecutor::updatePose(const nav_msgs::Odometry msg){
	//geometry_msgs::Point temp = msg.twist.twist.position;
	//ROS_INFO_STREAM(msg.pose.pose.orientation.x << "," << msg.pose.pose.orientation.y << "," << msg.pose.pose.orientation.z << "," << msg.pose.pose.orientation.w << "\n");
	rawZ = msg.pose.pose.orientation.z;
	currentAngle = (rawZ+1) * PI;//((rawZ + 1) * PI);
	if(justStarted){
		startAngle = currentAngle;
		justStarted = false;
		xStart = msg.pose.pose.position.x;
		yStart = msg.pose.pose.position.y;
	}
	xPred = msg.pose.pose.position.x - xStart;
	yPred = msg.pose.pose.position.y - yStart;
}

void PathExecutor::updateBumper(const kobuki_msgs::BumperEventConstPtr msg)
{
	//ROS_INFO_STREAM(msg->state);
	//Reset Program
	/////Quit Program
	//done = true;
	velo_message.linear.x = 0.0;
	velo_message.linear.y = 0.0;
	velo_message.linear.z = 0.0;
	velo_message.angular.x = 0.0;
	velo_message.angular.y = 0.0;
	velo_message.angular.z = 0.0;
	drive_pub.publish(velo_message);
	pathLog.close();
	state = 0;
}

void PathExecutor::updatePath(const std_msgs::String::ConstPtr& msg){
	//Parse through message
	const char* raw = msg->data.c_str();
	ROS_INFO_STREAM(raw);
	rapidjson::Document document;
	document.Parse(raw);
	assert(document.IsObject()); 
	for (rapidjson::Value::ConstMemberIterator itr = document.MemberBegin(); itr != document.MemberEnd(); ++itr){
		ROS_INFO_STREAM(ipAddress.compare(itr->name.GetString()));
		if(ipAddress.compare(itr->name.GetString()) == 0){
			//Data being recieved
			rapidjson::Value& robotPath = document[itr->name.GetString()];	
			for (rapidjson::SizeType i = 0; i < robotPath.Size(); i++){
				rapidjson::Value& rPoint =  robotPath[i];
				//ROS_INFO_STREAM(rPoint["X"].GetFloat());
    			pointVector.push_back(WayPoint(rPoint["X"].GetFloat() , rPoint["Y"].GetFloat()));
			}
			//tester();
			newDataRecieved = true;
			break;
		}
	}
	done = false;
}

bool PathExecutor::isRunning(){
	return(!done);
}

int PathExecutor::getRobotIP(){
	int currentSocket = socket(AF_INET , SOCK_DGRAM , 0);
	struct ifreq ifr;

	ifr.ifr_addr.sa_family = AF_INET;
	strncpy(ifr.ifr_name , "wlan0" , IFNAMSIZ-1);

	ioctl(currentSocket , SIOCGIFADDR , &ifr);

	close(currentSocket);

	ipAddress = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
	ROS_INFO_STREAM(ipAddress);
	return 0;
}

void PathExecutor::tester(){
	//Test JSON
	const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
    rapidjson::Document d;
    d.Parse(json);
    ROS_INFO_STREAM(d["project"].GetString());

    //Print Points
    for (std::vector<WayPoint>::const_iterator iterator = pointVector.begin(), end = pointVector.end(); iterator != end; ++iterator) {
    	WayPoint point = *iterator;
    	ROS_INFO_STREAM("X: " << point.getX() << " Y: " << point.getY());
    }
}

WayPoint PathExecutor::getLookAheadPoint(WayPoint currPoint){
	/*
		Assumptions
			1. pointVector starts at WayPoint(0,0)
			2. All points from there are jointly connected

	*/
	WayPoint out;
	WayPoint closestPoint; //Closest point
	double dist;
	double prevDist;
	bool first = true;
	bool foundLookAhead = false;
	double lookAheadInteg = 0;
	for (std::vector<WayPoint>::const_iterator iterator = pointVector.begin(), end = pointVector.end(); iterator != end; ++iterator) {
    	dist = std::sqrt(std::pow(iterator->X - currPoint.getX() , 2) + std::pow(iterator->Y - currPoint.getY() , 2));
    	//ROS_INFO_STREAM("POINT " << iterator->X << " " << iterator->Y << " DIST " << dist);
    	if(first){
    		closestPoint = *iterator;
    		out = *iterator;
    		prevDist = dist;
    		first = false;
    	}
    	else if(prevDist > dist){
    		closestPoint = *iterator;
    		out = *iterator;
    		prevDist = dist;
    		lookAheadInteg = 0;
    		foundLookAhead = false;
    	}
    	else if(!foundLookAhead){
    		double changeInDist = std::sqrt(std::pow(iterator->X - out.getX() , 2) + std::pow(iterator->Y - out.getY() , 2));
    		if(changeInDist + lookAheadInteg < lookAhead){
    			lookAheadInteg += changeInDist;
    			out = *iterator;
    		}
    		else{
    			//Interpolate point
    			double neededChange = lookAhead - lookAheadInteg;
    			double ratio = neededChange/changeInDist;

    			//double angle = std::atan2(iterator->Y - out.getY() , iterator->X - out.getX());
    			double nextX = out.getX() + (ratio * (iterator->X - out.getX()));//neededChange * std::cos(angle);
    			double nextY = out.getY() + (ratio * (iterator->Y - out.getY()));//neededChange * std::sin(angle);
    			/*
    			double nextX = out.getX() - ((neededChange * (out.getX() - iterator->X))/changeInDist);
    			double b = out.getY() - (slope * out.getX());
    			double nextY = (slope * nextX) + b;
    			*/
    			out = WayPoint(nextX , nextY);
    			foundLookAhead = true;
    		}
    	}
	}
	pathLog << xPred << "," << yPred << "," << currentAngle << "," << closestPoint.getX() << "," << closestPoint.getY() << "," << out.getX() << "," << out.getY() << "\n";
	return out;
}



/*
	Main Loop
*/

int main(int argc , char** argv)
{
	ros::init(argc , argv , "line_following");
	ros::NodeHandle n;
	
	std::vector<WayPoint> points;
	/*
	points.push_back(WayPoint(0.0,0.0));
	points.push_back(WayPoint(0.0,0.5));
	points.push_back(WayPoint(0.0,1.0));
	points.push_back(WayPoint(0.5,1.0));
	points.push_back(WayPoint(1.0,1.0));
	points.push_back(WayPoint(1.0,0.5));
	points.push_back(WayPoint(1.0,0.0));
	points.push_back(WayPoint(0.5,0.0));
	*/

	PathExecutor executor(n , points);
	 
	while(ros::ok() && executor.isRunning()){
		executor.stateMachine();
		ros::spinOnce();
	}
	
	return 0;
}