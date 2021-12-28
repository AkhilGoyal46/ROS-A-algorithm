#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <std_msgs/Int32MultiArray.h>
#include <cstdlib>
#include <math.h>
#include <vector>

class Drive{

private:
	ros::NodeHandle n;
	ros::Subscriber odom_sub;
	ros::Subscriber marker_sub;
	ros::Publisher drive_pub;
	
	geometry_msgs::Point currentPose, goalPose;
	geometry_msgs::Quaternion current_orientation;
	geometry_msgs::Twist state;
	std::vector<geometry_msgs::Point> targetPose;
	float seek_orientation;
	float current_angle;
	bool goalReached =false;
	int node = 0;
	bool dataRecieved=false;

public:
	Drive(){
		n=ros::NodeHandle("~");
		std::string odom_topic="/odom";
		std::string drive_topic="/cmd_vel" ;
		std::string marker_topic="/planning_viz" ;
		
		odom_sub = n.subscribe(odom_topic, 1, &Drive::odom_callback,this);
		marker_sub = n.subscribe(marker_topic, 1, &Drive::marker_callback,this);
		drive_pub= n.advertise<geometry_msgs::Twist>(drive_topic,10);
	}

	void odom_callback( const nav_msgs::Odometry & msg) {
		if ( dataRecieved){
		if( node <= targetPose.size()-1){
			goalPose=targetPose[node];
			currentPose=msg.pose.pose.position;
			//std::cout<<currentPose<<std::endl;
			current_orientation=msg.pose.pose.orientation;
			current_angle = 2*atan2(current_orientation.z,current_orientation.w);
			seek_orientation = atan2(goalPose.y - currentPose.y, goalPose.x - currentPose.x);
			if  (pow( (pow((goalPose.y - currentPose.y),2) + pow((goalPose.x - currentPose.x),2)), 0.5) > 0.1){
				state.angular.z= seek_orientation-current_angle;
				if (abs(state.angular.z) < 0.01){state.linear.x = 1.0;}
				else{state.linear.x = 0.0;}
				drive_pub.publish(state);
		
			}else{node +=1;}
		}else{goalReached=true;}
		
		if(goalReached){state.angular.z=0; state.linear.x = 0.0;drive_pub.publish(state);}
		
	}}

	void marker_callback( const visualization_msgs::Marker & msg) {
	targetPose = msg.points;
	if (targetPose.size()>0){dataRecieved = true;}
	}




};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "drive");
    Drive dv;
    ros::spin();
    return 0;
}
