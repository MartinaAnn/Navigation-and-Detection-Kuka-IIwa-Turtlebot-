#include "ros/ros.h"
#include <aruco/aruco.h>
#include "boost/thread.hpp"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <aruco_ros/aruco_ros_utils.h>
#include <aruco_msgs/MarkerArray.h>
//#include <aruco_msgs/Marker.h>
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
	//	ros::Subscriber  _marker_sub;
		//vector<aruco_msgs::Marker> _marker;
		//aruco_msgs::MarkerArray::Ptr _marker_msg;
		std::vector<Coord> _room;
  
    public:
		SEARCH_OBJ(); 
		void chooseinput();
		void run();
        void move(float x_des, float y_des, float w_des);
		//void turn();
		void check();
		bool done = false;
		int marker_id_des=0;
		bool found = false;
		bool center =false;
		bool turned=false;
		struct Coord kuka;

	
		//bool pubmark= _marker_pub.getNumSubscribers()>0;
		//aruco_msgs::Marker marker_id;
			

};

SEARCH_OBJ::SEARCH_OBJ(){
struct Coord first_room ;
		first_room.x=0.0;
		first_room.y=0.0;
		first_room.w=1.0;

		struct Coord second_room;
		second_room.x=-6.3;
		second_room.y=0.0;
		second_room.w=-3.0;

		struct Coord third_room;
		third_room.x=-6.3;
		third_room.y=3.0;
		third_room.w=-1.0;

		struct Coord fourth_room;
		fourth_room.x=0.0;
		fourth_room.y=3.0;
		fourth_room.w=1.0;

		kuka.x=-3.0;
		kuka.y=5.5;
		kuka.w=1.0;

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

	done = true;	
}

/*void SEARCH_OBJ::turn(){
	while(<360)
	gira;
	if(pubmark)
	break;
	if(posa==360°)
	turned=true;
}
*/
void SEARCH_OBJ::move(float x_des, float y_des, float w_des){

//tell the action client that we want to spin a thread by default
 MoveBaseClient ac("move_base", true);

//wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }  
	 move_base_msgs::MoveBaseGoal goal;

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
	   }
     else
       ROS_INFO("The base failed to move for some reason");
	
   
}

void SEARCH_OBJ::check(){

	//_marker_sub = 
	//_marker_msg = aruco_msgs::MarkerArray::Ptr(new aruco_msgs::MarkerArray());
	
	//aruco_msgs::Marker & marker = _marker_msg->markers.at(_marker.size());
    if(done==true) { 
		int i=0;
		while (i<4){
       	 move(_room[i].x, _room[i].y, _room[i].w);
		
		 /* if(!pubmark){*/
				if(center && i!=3/*&& turned*/)
				i++;
				/*else if (center && !turned)
				turn();*/
		/*  }*/


      	/*  else if (pubmark)
       	     {*/
				else if(/*marker.id==marker_id_des*/ i==3){	
				move(kuka.x,kuka.y,kuka.w);	
				//cout << id <<endl;
				ROS_INFO("Turtlebot found the tool, now it will reach Kuka Iiwa.");
				found=true;
				break;
				}
			/*	else if(marker.id!=marker_id_des && center && turned)
				i++;          
         	   
         	/*   }*/
        	
   		 }
	}
}
void SEARCH_OBJ::run() {
	boost::thread check_t( &SEARCH_OBJ::check, this);
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "search_obj"); //node name

	SEARCH_OBJ so;

	so.chooseinput();
	so.run();
	return 0;
}