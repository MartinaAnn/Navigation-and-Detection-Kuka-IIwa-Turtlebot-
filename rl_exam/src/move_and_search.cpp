#include "ros/ros.h"
#include "rl_exam/service.h"
#include <iostream>
#include <sstream>
#include <aruco/aruco.h>
#include "boost/thread.hpp"
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt32MultiArray.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	
class SEARCH_OBJ{
    private:
			struct Coord{
			float x;
			float y;
			float w;
		};
		ros::NodeHandle _nh;
		ros::Subscriber  _marker_sub;
		ros::Publisher _vel_pub;
		const aruco_msgs::MarkerArray::ConstPtr markerpoint;
		geometry_msgs::Twist cmd_vel;
		std::vector<Coord> _room;
		aruco_msgs::MarkerArray marker_msg;
		float x_des, y_des, w_des;
		float rot_vel=0.8;
		int marker_id_des=0;
		struct Coord kuka, first_room, second_room, third_room, fourth_room;
		
  
    public:
		SEARCH_OBJ(); 
		void chooseinput();
		void move(float x_des, float y_des, float w_des);
		void run();
       	void rotate();
		void check();
		bool done = false;
		bool center =false;
		bool turned=false;
		bool pubmark= false;
	
		void markerMsgsCallback(const aruco_msgs::MarkerArray::ConstPtr &markerpoint);
		void reach_goal();
};

SEARCH_OBJ::SEARCH_OBJ(){
	//ASSIGNING COORDINATES TO ROOMS

		first_room.x=0.0;
		first_room.y=0.0;
		first_room.w=1.0;

		second_room.x=-6.3;
		second_room.y=0.0;
		second_room.w=0.2;

		third_room.x=-6.3;
		third_room.y=3.0;
		third_room.w=-1.0;

		fourth_room.x=0.0;
		fourth_room.y=3.0;
		fourth_room.w=1.0;

		kuka.x=-3.0;
		kuka.y=5.8;
		kuka.w=0.5;

		//CREATING A VECTOR OF ROOMS

		_room.push_back(first_room);
		_room.push_back(second_room);
		_room.push_back(third_room);
		_room.push_back(fourth_room);
}

void SEARCH_OBJ::chooseinput() {

	int num;

	cout << "What tool do you want to search for? " << endl;
	cout << "[1]: Hammer" << endl;
	cout << "[2]: Screwdriver" << endl;
	cout << "[3]: Wrench" << endl;
	cout << "[4]: Drill" << endl;

    cin >> num;

    if( num== 1 ) {
			cout << "Turtlebot will search for the hammer" << endl;
			marker_id_des=16;
	}
	else if( num == 2) {
			cout << "Turtlebot will search for the screwdriver" << endl;
			marker_id_des=19;
	}
	else if( num == 3) {
			cout << "Turtlebot will search for the wrench" << endl;
			marker_id_des=24;
	}
	else if( num == 4) {
			cout << "Turtlebot will search for the drill" << endl;
			marker_id_des=35;
	};
	return;
}

void SEARCH_OBJ::rotate(){
	ros::Rate r(10);
	_vel_pub = _nh.advertise<geometry_msgs::Twist >("turtlebot3/cmd_vel", 1);
	cout << "Start rotation" << endl;
	cmd_vel.angular.z = rot_vel;
   	while(marker_msg.markers.size() == 0 ) {
			_vel_pub.publish( cmd_vel );
		
				r.sleep();
	}
	pubmark=false;
	if( marker_msg.markers[0].id==marker_id_des){	
				reach_goal();
				
		}	
		return;
}

void SEARCH_OBJ::move(float x_des, float y_des, float w_des){

//tell the action client that we want to spin a thread by default
 MoveBaseClient ac("move_base", true);

//wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }  
	 move_base_msgs::MoveBaseGoal goal;
	 center=false;
     goal.target_pose.header.frame_id = "map";
     goal.target_pose.header.stamp = ros::Time::now();
   
     goal.target_pose.pose.position.x = x_des;
	 goal.target_pose.pose.position.y = y_des;             
	 goal.target_pose.pose.orientation.w = w_des;
   
     ROS_INFO("Sending goal");
     ac.sendGoal(goal);
   
     ac.waitForResult();
   
     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       { ROS_INFO("Turtlebot is in the middle of the room");
		center=true;
		if(goal.target_pose.pose.position.x==kuka.x && goal.target_pose.pose.position.y == kuka.y){
				cout <<"end" <<endl;
				//CLIENT
						ros::NodeHandle n;
	ros::Rate loop_rate(10);
	ros::ServiceClient client =
	n.serviceClient<rl_exam::service>("service");

		rl_exam::service srv;
		std::stringstream ss;
		ss << "Starting from Here";
		srv.request.in = ss.str();
		if (client.call(srv)) {
			cout << "From Client: ["<<	srv.request.in << "], Server says [" << srv.response.out << "]" << endl;
		}
		else {
			ROS_ERROR("Failed to call service");
		}
		ros::spinOnce();
		loop_rate.sleep();
	
	}
		
		
		 	
		 }
     else
       ROS_INFO("The base failed to move for some reason");
   
}

void SEARCH_OBJ::reach_goal()
{
ROS_INFO("Turtlebot found the tool, now it will reach Kuka Iiwa.");
				usleep(1000000);
				move(kuka.x,kuka.y,kuka.w);	
}

void SEARCH_OBJ::check(){
	
	_marker_sub = _nh.subscribe("/aruco_marker_publisher/markers", 10, &SEARCH_OBJ::markerMsgsCallback, this);
	
	
		for (int i=0; i<4; i++){
			  
       	 move(_room[i].x, _room[i].y, _room[i].w);
		 if (center){
			marker_msg.markers.clear();
			cout<< "valore size iniziale: " << marker_msg.markers.size() <<endl;
	  		pubmark=true;
			usleep(1000000);
		    if(marker_msg.markers.size() > 0) 
       	     {
				cout<<"valore dopo pubmark=true: " << marker_msg.markers.size() << endl;
				pubmark=false;
				if( marker_msg.markers[0].id==marker_id_des){	
				reach_goal();
				break;
				}
				}        
        	else{
				rotate();
				}
			
		  }
		}
   		
		return;
	}

void SEARCH_OBJ::markerMsgsCallback(const aruco_msgs::MarkerArray::ConstPtr &markerpoint){
	if(pubmark){
	marker_msg.markers = markerpoint -> markers;
	cout << marker_msg.markers[0].id << endl;
	}
}

void SEARCH_OBJ::run() {
	boost::thread check_t( &SEARCH_OBJ::check, this);
	//boost::thread rotate_t(&SEARCH_OBJ::rotate,this);
	//boost::thread move_t(&SEARCH_OBJ::move, this, x_des, y_des, w_des);
	boost::thread markerMsgsCallback_t(&SEARCH_OBJ::markerMsgsCallback, this, markerpoint);
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "search_obj"); //node name
	SEARCH_OBJ so;
	so.chooseinput();
	so.run();
	

	return 0;
}
