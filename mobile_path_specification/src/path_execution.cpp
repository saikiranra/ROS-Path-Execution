#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <algorithm>
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
#include <cstdlib>
#include "rapidjson/document.h"



/*
	WayPoint Class
*/

class WayPoint{
	public:
		WayPoint(double xPoint, double yPoint);
		WayPoint();
		~WayPoint();
		double getX() const;
		double getY() const;
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

double WayPoint::getX() const{
	return(X);
}

double WayPoint::getY() const{
	return(Y);
}

/*
	PathSegment Class
*/

class PathSegment{
	public:
		PathSegment(WayPoint A , WayPoint B);
		PathSegment();
		~PathSegment();
		WayPoint getClosestPoint(WayPoint currentPoint) const;
		WayPoint getPointOnExtendedLine(double distanceSquaredLeft) const;
		double distanceSquared(WayPoint one , WayPoint two) const;
		double getLengthSquared() const;
		bool sameAsEnd(WayPoint A) const;
		WayPoint start;
		WayPoint end;
		double lengthSquared;
		double dx;
		double dy;
		
};

PathSegment::PathSegment(WayPoint A , WayPoint B){
	start = A;
	end = B;
	dx = end.getX() - start.getX();
	dy = end.getY() - start.getY();
	lengthSquared = distanceSquared(A , B);
}

PathSegment::PathSegment(){
	start = WayPoint(0,0);
	end = WayPoint(0,0);
	dx = end.getX() - start.getX();
	dy = end.getY() - start.getY();
	lengthSquared = distanceSquared(start , end);
}

PathSegment::~PathSegment(){

}

double PathSegment::distanceSquared(WayPoint one , WayPoint two) const{
	return std::pow(one.getX() - two.getX() , 2) + std::pow(one.getY() - two.getY() , 2);
}

bool PathSegment::sameAsEnd(WayPoint A) const{
	return((A.getX() == end.getX()) && (A.getY() == end.getY()));
}

WayPoint PathSegment::getClosestPoint(WayPoint currentPoint) const{
	if(lengthSquared == 0){
		return end;
	}
	double t = ((currentPoint.getX() - start.getX())*dx + (currentPoint.getY() - start.getY())*dy)/lengthSquared;
	t = std::max(0.0 , std::min(1.0 , t));

	return WayPoint(start.getX() + t * dx , start.getY() + t * dy);
}

WayPoint PathSegment::getPointOnExtendedLine(double distanceSquaredLeft) const{
	//Meant to be used at the end of a path sequence when the robot is still traveling on the line but the lookahead distance has exceded the path
	double t = std::sqrt(distanceSquaredLeft/lengthSquared);
	//ROS_INFO_STREAM("POINT ON EXTENDED: X " << end.getX() + t*dx << " Y " << end.getY() + t*dy);
	return WayPoint(end.getX() + t*dx , end.getY() + t*dy);
}

double PathSegment::getLengthSquared() const{
	return lengthSquared;
}

/*
	Path Executor Class
*/

class PathExecutor{
	public:
		PathExecutor(std::string name, ros::NodeHandle &n, std::vector<WayPoint> wps);
		void updatePose(const nav_msgs::Odometry msg);
		void updateBumper(const kobuki_msgs::BumperEventConstPtr msg);
		void updatePath(const std_msgs::String::ConstPtr& msg);
		void stateMachine();
		void resetRobot();
		void convertPointsToSegments();
		void tester();
		void endPath();
		int getRobotIP();
		bool isRunning();
		double getAdaptiveLookahead(WayPoint currentPoint , WayPoint goalPoint);
		WayPoint getLookAheadPoint(WayPoint currentLocation);
		WayPoint getNewLookAheadPoint(WayPoint currentLocation);
	private:
		ros::NodeHandle node;
		ros::Subscriber pose_sub, bumper_sub , path_sub;
		ros::Publisher drive_pub;
		std::vector<WayPoint> pointVector;
		std::vector<PathSegment> segmentVector;
		geometry_msgs::Pose pose_message;
		geometry_msgs::Twist velo_message;
		WayPoint previousSetPoint;
		WayPoint currentSetPoint;
		std::string robotName;
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
		double previousGamma;
		double currentLookAheadSquared; //Used for the adaptive lookahead model
		std::string debugString;
		WayPoint current;
		WayPoint goalPoint;
		double goalPointRX;
		double goalPointRY;
		const double PI  = 3.141592653589793238463;
		const double lookAhead = 0.03; //0.04;
		const double lookAheadSquared = 0.0016;//0.0016;
		const double lookAheadSmallSquared = 0.0004;
		const double lookAheadLargeSquared = 0.01;
		const double width = 0.23;
		const double gammaLimit = 3;
		const double maxAngularAccel = 3;
		ros::Time timePathStart;
		ros::Time currentTime;
		ros::Time previousTime;
		std::ofstream pathLog;
		std::ofstream speedLog;



};

PathExecutor::PathExecutor(std::string name, ros::NodeHandle &n , std::vector<WayPoint> wps):pointVector(wps){
	node = n;
	robotName = name;
	state = 0;
	resetRobot();
	pose_sub = node.subscribe(robotName+"/odom", 10, &PathExecutor::updatePose, this);
	bumper_sub = node.subscribe(robotName+"/mobile_base/events/bumper", 10, &PathExecutor::updateBumper, this);
	path_sub = node.subscribe("robot_paths" , 10 , &PathExecutor::updatePath , this);
	drive_pub = node.advertise<geometry_msgs::Twist>(robotName+"/mobile_base/commands/velocity", 1, true);
	ROS_INFO_STREAM("Path Executor Starting For Robot " << name << "\n");
}

void PathExecutor::resetRobot(){
	pointVector.clear();
	segmentVector.clear();
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
	double gammaExperiment;
	double accelExperiment;
	currentTime = ros::Time::now();
	ros::Duration timeDelta = currentTime - previousTime;
	ros::Duration diff = currentTime - timePathStart;

	PathSegment currentSegment;
	std::ostringstream test;
	std::string t;
	switch(state){
		case 0:
			//Reset State
			//ROS_INFO_STREAM("State 0");
			getRobotIP(); //Reset IP address
			resetRobot();
			velo_message.linear.x = 0;
			velo_message.angular.z = 0;
			previousGamma = 0;
			state = 1;
			break;
		case 1:
			//Wait for Data
			if(newDataRecieved){
				convertPointsToSegments();
				pathLog.open("pathLog.txt");
				pathLog << "Robot X,Robot Y,Robot Angle,Close X,Close Y,Goal X,Goal Y\n";
				speedLog.open("speedLog.txt");
				speedLog << "time,Linear,Angular\n";
				timePathStart = ros::Time::now();
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
			goalPoint = getNewLookAheadPoint(current);//getLookAheadPoint(current);
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
			speed = 0.15;

			//currentSegment = PathSegment(current , goalPoint);
			if(std::abs(crossProduct) > 0 && std::abs(crossTerm) > 0){
				radius = 0.5 * std::abs(dx * dx + dy * dy)/crossTerm;
				//radius = std::copysign(std::sqrt(currentSegment.getLengthSquared()) , crossTerm);

				gamma = speed / radius;
				if(crossProduct > 0){
					gamma = -gamma;
				}
				
				if(std::abs(gamma) > gammaLimit){ //Limiting turn speed for consistency
					gamma =  std::copysign(gammaLimit , gamma);
					speed = std::copysign(gamma * radius , 1);
				}
				/*if(std::abs(accelExperiment) > maxAngularAccel){
					gamma = previousGamma + std::copysign(maxAngularAccel * timeDelta.toSec() , gamma);
					speed = gamma * radius;
				}*/

			}
			state = 2;
			if(segmentVector.size() < 2){
				endPath();
			}
			
			//ROS_INFO_STREAM("CURRENT POINT(" << xPred << "," << yPred << ") GOAL POINT (" << goalPoint.getX() << "," << goalPoint.getY() << ") GAMMA " << gamma << " RADIUS " << radius << "SEG LENGTH " << segmentVector.size());
			
			//test << "CURRENT POINT(" << xPred << "," << yPred << ") GOAL POINT (" << goalPoint.getX() << "," << goalPoint.getY() << ") GAMMA " << gamma << " RADIUS " << radius << "SEG LENGTH " << segmentVector.size();
			
			
				
			accelExperiment = (gamma - previousGamma)/timeDelta.toSec();

			/*
			test << "Computer Accel " << accelExperiment << "SEG LENGTH " << segmentVector.size();

			t = test.str();
			if(t != debugString){
				debugString = t;
				ROS_INFO_STREAM(t);
			}
			*/

			velo_message.linear.x = speed;
			velo_message.angular.z = gamma;

			
			speedLog << diff.toSec() << "," << speed << "," << gamma << "\n";
			break;
	}
	drive_pub.publish(velo_message);
	previousTime = currentTime;
	previousGamma = gamma;
}

void PathExecutor::updatePose(const nav_msgs::Odometry msg){
	//geometry_msgs::Point temp = msg.twist.twist.position;
	//ROS_INFO_STREAM(msg.pose.pose.orientation.x << "," << msg.pose.pose.orientation.y << "," << msg.pose.pose.orientation.z << "," << msg.pose.pose.orientation.w << "\n");
	rawZ = msg.pose.pose.orientation.z;

	if(justStarted){
		//startAngle = (rawZ+1) * PI;
		justStarted = false;
		xStart = msg.pose.pose.position.x;
		yStart = msg.pose.pose.position.y;
	}

	//currentAngle = ((rawZ+1) * PI) - startAngle;//((rawZ + 1) * PI);
	currentAngle = (rawZ+1) * PI;
	
	xPred = msg.pose.pose.position.x - xStart;
	yPred = msg.pose.pose.position.y - yStart;
}

void PathExecutor::updateBumper(const kobuki_msgs::BumperEventConstPtr msg)
{
	//ROS_INFO_STREAM(msg->state);
	//Reset Program
	endPath();
}

void PathExecutor::endPath(){
	velo_message.linear.x = 0.0;
	velo_message.linear.y = 0.0;
	velo_message.linear.z = 0.0;
	velo_message.angular.x = 0.0;
	velo_message.angular.y = 0.0;
	velo_message.angular.z = 0.0;
	drive_pub.publish(velo_message);
	pathLog.close();
	speedLog.close();
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

void PathExecutor::convertPointsToSegments(){
	//segmentList
	bool notFirst = false;
	WayPoint prev;
	for (std::vector<WayPoint>::const_iterator iterator = pointVector.begin(), end = pointVector.end(); iterator != end; ++iterator) {
		if(notFirst){
			segmentVector.push_back(PathSegment(prev , *iterator));
		}
		else{
			notFirst = true;
		}
		prev = *iterator;
	}
	//ROS_INFO_STREAM("SIZE " << segmentVector.size());
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

double PathExecutor::getAdaptiveLookahead(WayPoint currentPoint , WayPoint goalPoint){
	//double lateralSquaredError = std::pow(currentPoint.getX() - goalPoint.getX() , 2) + std::pow(currentPoint.getY() - goalPoint.getY() , 2);
	//The smaller the error, the larger the lookahead distance
	return(lookAheadSquared);
}

WayPoint PathExecutor::getNewLookAheadPoint(WayPoint currPoint){
	std::vector<PathSegment>::iterator iterator = segmentVector.begin();
	std::vector<PathSegment>::iterator end = segmentVector.end();
	PathSegment lastPathSeg;
	int eraseIndex = -1;
	WayPoint closest;
	WayPoint lastMeasure;
	WayPoint lookAheadPoint;
	WayPoint debug;
	int debugState = 100;
	bool pointNotFound = true;
	int lookCase = 0;
	double distanceSquaredAccum  = 0;
	while(iterator != end && pointNotFound){
		debug = (*iterator).end;
		
		switch(lookCase){
			case 0: //Looking for closest point
				closest = (*iterator).getClosestPoint(currPoint);
				if((*iterator).sameAsEnd(closest)){ // Check if the closest point is the end of the segment to find when to progress
					++eraseIndex;
					++iterator;
					//iterator = segmentVector.erase(iterator);

					if(debugState != 0 && false){
						ROS_INFO_STREAM("POINT " << debug.getX() << " " << debug.getY() << " CASE " << lookCase);
						ROS_INFO_STREAM("Same As End");
						ROS_INFO_STREAM("SEG VECT SIZE " << segmentVector.size());
						debugState = 0;
					}
				}
				else{
					lookCase = 1;
					lastMeasure = closest;
					//Change lookahead distance
					currentLookAheadSquared = getAdaptiveLookahead(current , closest);

					if(debugState != 1 && false){
						ROS_INFO_STREAM("POINT " << debug.getX() << " " << debug.getY() << " CASE " << lookCase);
						ROS_INFO_STREAM("At Right Seg");
						ROS_INFO_STREAM("SEG VECT SIZE " << segmentVector.size());
						debugState = 1;
					}
				}
				break;
			case 1: //Looking for lookahead point
				double distanceSquaredLeftOnSegment = (*iterator).distanceSquared(lastMeasure , (*iterator).end);
				if(distanceSquaredAccum + distanceSquaredLeftOnSegment >= currentLookAheadSquared){ //Point on this seg
					double neededChange = currentLookAheadSquared - distanceSquaredAccum;
					double ratio = std::sqrt(neededChange/distanceSquaredLeftOnSegment);

					double nextX = lastMeasure.getX() + (ratio * ((*iterator).end.getX() - lastMeasure.getX()));
					double nextY = lastMeasure.getY() + (ratio * ((*iterator).end.getY() - lastMeasure.getY()));
					
					lookAheadPoint = WayPoint(nextX , nextY);
					pointNotFound = false;
					if(debugState != 2 && false){
						ROS_INFO_STREAM("POINT " << debug.getX() << " " << debug.getY() << " CASE " << lookCase);
						ROS_INFO_STREAM("Found Point");
						ROS_INFO_STREAM("SEG VECT SIZE " << segmentVector.size());
						debugState = 2;
					}

				}
				else{ //Point after this seg
					distanceSquaredAccum += distanceSquaredLeftOnSegment;
					lastMeasure = (*iterator).end;
					lastPathSeg = *iterator;
					++iterator;
					if(debugState != 3 && false){
						ROS_INFO_STREAM("POINT " << debug.getX() << " " << debug.getY() << " CASE " << lookCase);
						ROS_INFO_STREAM("Point After Seg");
						ROS_INFO_STREAM("SEG VECT SIZE " << segmentVector.size());
						debugState = 3;
					}
				}
				break;

		}

	}
	
	if(pointNotFound){
		//At last point ;
		lookAheadPoint = lastPathSeg.getPointOnExtendedLine(currentLookAheadSquared - distanceSquaredAccum);
	}
	/*
	std::ostringstream test;
	test << "POINT " << lookAheadPoint.getX() << " " << lookAheadPoint.getY() << " SEG VECT SIZE " << segmentVector.size();
	std::string t = test.str();
	if(t != debugString){
		debugString = t;
		ROS_INFO_STREAM(t);
	}
	*/
	if(eraseIndex != -1){
		segmentVector.erase(segmentVector.begin() , segmentVector.begin() + eraseIndex);
	}
	if(! std::isnan(lookAheadPoint.getX())){
		pathLog << xPred << "," << yPred << "," << currentAngle << "," << closest.getX() << "," << closest.getY() << "," << lookAheadPoint.getX() << "," << lookAheadPoint.getY() << "\n";
	
	}
	return(lookAheadPoint);
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
	std::string rname;
	//ros::param::param<std::string>("RobotName", rname, "noname");
	rname = std::getenv("TURTLEBOT_NAME");
	ros::init(argc , argv ,  rname+"_Path_Execution");
	ros::NodeHandle n;
	//n.getParam("RobotName" , rname);
	
	std::vector<WayPoint> points;

	ros::Rate loop_rate(5);
	PathExecutor executor(rname, n , points);
	 
	while(ros::ok() && executor.isRunning()){
		executor.stateMachine();
		ros::spinOnce();
	}
	
	return 0;
}