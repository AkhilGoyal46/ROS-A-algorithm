#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <std_msgs/Int32MultiArray.h>
#include <cstdlib>
#include <math.h>
#include <vector>

class Planning {

private:
	
	ros::NodeHandle n;
	
	
	ros::Subscriber odom_sub; 
	visualization_msgs::Marker path;
	ros::Publisher marker_pub;
	
	geometry_msgs::Point currentPose , goal , c_idx, g_idx, next_pos;
	
	float map[20][18]={0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1};
       
       float h[20][18]= {0};
       float g[20][18]= {0};
       float f[20][18]= {0};
       int node[20][18]= {0};
       int o[20][18]= {0};
       char state[20][18]= {'0'};
       int s_node=0;
       float ds , dd;
       
       int w=0;
       
       
public:
	Planning(){
		
		n=ros::NodeHandle("~");
		//n.param<float>("Goal", goal , 4.0);
		std::string odom_topic="/odom";
		std::string marker_topic = "/planning_viz";
		
		
		marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic, 1);
		
		path.header.frame_id = "base_link";
		path.header.stamp = ros::Time::now();
		path.ns = "Path";
		path.action = visualization_msgs::Marker::ADD;
		path.pose.orientation.w = 1.0;
		path.id = 0;
		path.type = visualization_msgs::Marker::LINE_STRIP;
		path.scale.x = 0.3;
		
		path.color.r = 1.0;
		path.color.a=1.0;
		
		odom_sub = n.subscribe(odom_topic, 1, &Planning::odom_callback,this);
		//std::cout<<map<<std::endl;
		
		//goal.x=4.5;
       	//goal.y=9;
       	
       	n.getParam("goalx", goal.x);
       	n.getParam("goaly", goal.y);
		g_idx.x= 9 - ceil(goal.y);
		g_idx.y= 8 + ceil(goal.x);
		ds = 1;
		dd = 1.4; //pow(2,0.5);
		
	}
	
	void odom_callback( const nav_msgs::Odometry & msg){
		currentPose=msg.pose.pose.position;
		c_idx.x= 9 - ceil(currentPose.y);
		c_idx.y= 8 + ceil(currentPose.x);
		//c_idx.x= 0;
		//c_idx.y= 13;
		//std::cout<< goal<<std::endl;
		get_path(c_idx);
		
		
		
		//for( int i = 0 ; i< 20 ; i++){
		//	for( int j = 0 ; j< 18 ; j++){
		//		std::cout<<node[i][j]<< "\t";
				
		//	}
		//	std::cout<<std::endl;
		//}std::cout<<std::endl;
		
		//for( int i = 0 ; i< 20 ; i++){
		//	for( int j = 0 ; j< 18 ; j++){
		//		std::cout<<f[i][j]<< "\t";
				
		//	}
		//	std::cout<<std::endl;
		//}std::cout<<std::endl;
		
		if(w<1){
		
		while(!(( c_idx.x == g_idx.x) && ( c_idx.y == g_idx.y))){
		c_idx = getNextPos(c_idx);
		//std::cout<< c_idx<<std::endl;
		
		
		next_pos.x= c_idx.y -8;
		next_pos.y= 9 - c_idx.x;
		//std::cout<< next_pos<<std::endl;
		s_node =0;
		
		path.points.push_back(next_pos);
		w = 1;
		}
		}
		clear_data();
		marker_pub.publish(path);
		
		//path.points.clear();
		
		
		
		
		
	}
	
	void get_path(geometry_msgs::Point P){
		int i = P.x ;
		int j = P.y;
		s_node +=1;
		node[i][j] = s_node;
		if(!(( P.x == g_idx.x) && ( P.y == g_idx.y)) ){
		state[i][j]='c';
		
		//node[i][j] = o[i][j] + 1;
		int nod = node[i][j];
		float cost = g[i][j];
		//state[i-1][j+1]='c';
		float min_cost=999999;
		int next_node[2] = {0 , 0} ;
		
		geometry_msgs::Point P1;
		
		P1.x = i;
		P1.y = j;
		
		if(if_valid(i-1,j-1)){//std::cout<<"1";
			set_val(i-1,j-1, cost , dd, nod);
			if(f[i-1][j-1]<min_cost){
				
				P1.x = i-1;
				P1.y = j-1;
				min_cost = f[i-1][j-1] ;
			}
		}
		if(if_valid(i-1,j)){//std::cout<<"2";
			set_val(i-1,j, cost , ds, nod);
			if(f[i-1][j]<min_cost){
				
				P1.x = i-1;
				P1.y = j;
				min_cost = f[i-1][j];
			}
		}
		if(if_valid(i-1,j+1)){//std::cout<<"3";
			set_val(i-1,j+1, cost , dd, nod);
			if(f[i-1][j+1]<min_cost){
				
				P1.x = i-1;
				P1.y = j+1;
				min_cost = f[i-1][j+1] ;
			}
		}
		if(if_valid(i,j+1)){//std::cout<<"4";
			set_val(i,j+1, cost , ds, nod);
			if(f[i][j+1]<min_cost){
				
				P1.x = i;
				P1.y = j+1;
				min_cost = f[i][j+1] ;
			}
		}
		if(if_valid(i+1,j+1)){//std::cout<<"5";
			set_val(i+1,j+1, cost , dd, nod);
			if(f[i+1][j+1]<min_cost){
				
				P1.x = i+1;
				P1.y = j+1;
				min_cost = f[i+1][j+1] ;
			}
		}
		if(if_valid(i+1,j)){//std::cout<<"6";
			set_val(i+1,j, cost , ds, nod);
			if(f[i+1][j]<min_cost){
				
				P1.x = i+1;
				P1.y = j;
				min_cost = f[i+1][j] ;
			}
		}
		if(if_valid(i+1,j-1)){//std::cout<<"7";
			set_val(i+1,j-1, cost , dd, nod);
			if(f[i+1][j-1]<min_cost){
				
				P1.x = i+1;
				P1.y = j-1;
				min_cost = f[i+1][j-1] ;
			}
			
		}
		if(if_valid(i,j-1)){//std::cout<<"8";
			set_val(i,j-1, cost , ds, nod);
			if(f[i][j-1]<min_cost){
				
				P1.x = i;
				P1.y = j-1;
				min_cost = f[i][j-1] ;
			}
		}
		
		
		//std::cout<< min_cost<<std::endl;
		
		
		//std::cout<<P1<<std::endl;
		
		if(!((P1.x == i) && (P1.y == j))){get_path(P1);} else {//std::cout<< "I am Stuck!"<< std::endl;
		}
		
	
	}else{//std::cout<< "Target Reached"<< std::endl;
	}
	}
	
	bool if_valid(int i , int j){
		if ((i>=0 && i<20)&&(j>=0 && j<18)&&(map[i][j] != 1)&&(state[i][j] != 'c')){return true;}
		else {return false;}
	
	}
	
	void set_val(int i, int j, float c, float ci , int nod){
		//if ( state[i][j] != 'o'){
			state[i][j] = 'o';
			o[i][j] = nod;
			h[i][j] = get_distance(i,j);
			g[i][j] = c + ci ;
			f[i][j] = h[i][j] + 0.7*g[i][j];
			
		//}	
	
	}
	
	float get_distance(int i,int j ){
		float gx = g_idx.x;
		float gy = g_idx.y;
		float MAx = std::max(abs(i-gx), abs(j-gy));
		float MIn= std::min(abs(i-gx), abs(j-gy));
		
		return (MIn*dd) + (MAx - MIn);
	}
	
	void clear_data(){
		
		for( int i = 0 ; i< 20 ; i++){
			for( int j = 0 ; j< 18 ; j++){
				h[i][j]=0;
				
			}
			
		}
		
		for( int i = 0 ; i< 20 ; i++){
			for( int j = 0 ; j< 18 ; j++){
				g[i][j]=0;
				
			}
			
		}
		
		for( int i = 0 ; i< 20 ; i++){
			for( int j = 0 ; j< 18 ; j++){
				f[i][j]=0;
				
			}
			
		}
		
		for( int i = 0 ; i< 20 ; i++){
			for( int j = 0 ; j< 18 ; j++){
				node[i][j]=0;
				
			}
			
		}
		
		for( int i = 0 ; i< 20 ; i++){
			for( int j = 0 ; j< 18 ; j++){
				o[i][j]=0;
				
			}
			
		}
		
		for( int i = 0 ; i< 20 ; i++){
			for( int j = 0 ; j< 18 ; j++){
				state[i][j]='0';
				
			}
			
		}
	
	
	}
	
	
	geometry_msgs::Point getNextPos(geometry_msgs::Point P){
		int i =P.x;
		int j =P.y;
		geometry_msgs::Point P1;
		P1=P;
		int hi_val=node[i][j];
		if(if_isvalid(i-1,j-1)){//std::cout<<"1";
			if(node[i-1][j-1]>hi_val){
				P1.x=i-1;
				P1.y=j-1;
				hi_val= node[i-1][j-1];
			}
			
		}
		if(if_isvalid(i-1,j)){//std::cout<<"2";
			if(node[i-1][j]>hi_val){
				P1.x=i-1;
				P1.y=j;
				hi_val= node[i-1][j];
			}
		}
		if(if_isvalid(i-1,j+1)){//std::cout<<"3";
			if(node[i-1][j+1]>hi_val){
				P1.x=i-1;
				P1.y=j+1;
				hi_val= node[i-1][j+1];
			}
		}
		if(if_isvalid(i,j+1)){//std::cout<<"4";
			if(node[i][j+1]>hi_val){
				P1.x=i;
				P1.y=j+1;
				hi_val= node[i][j+1];
			}
			
		}
		if(if_isvalid(i+1,j+1)){//std::cout<<"5";
			if(node[i+1][j+1]>hi_val){
				P1.x=i+1;
				P1.y=j+1;
				hi_val= node[i+1][j+1];
			}
		}
		if(if_isvalid(i+1,j)){//std::cout<<"6";
			if(node[i+1][j]>hi_val){
				P1.x=i+1;
				P1.y=j;
				hi_val= node[i+1][j];
			}
			
		}
		if(if_isvalid(i+1,j-1)){//std::cout<<"7";
			if(node[i+1][j-1]>hi_val){
				P1.x=i+1;
				P1.y=j-1;
				hi_val= node[i+1][j-1];
			}
			
			
		}
		if(if_isvalid(i,j-1)){//std::cout<<"8";
			if(node[i][j-1]>hi_val){
				P1.x=i;
				P1.y=j-1;
				hi_val= node[i][j-1];
			}
		}
	
		return P1;
	}
	
	bool if_isvalid(int i , int j){
	if ((i>=0 && i<20)&&(j>=0 && j<18)){return true;}
		else {return false;}
	
	} 
	
	
	
	
};

int main (int argc, char **argv){
	
	ros::init(argc, argv, "planning");
	Planning pl;
	ros::spin();
	return 0;
}
