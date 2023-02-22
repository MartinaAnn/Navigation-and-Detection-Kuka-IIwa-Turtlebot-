#include "ros/ros.h"
#include <aruco/aruco.h>
#include "boost/thread.hpp"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <aruco_ros/aruco_ros_utils.h>
//#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt32MultiArray.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SEARCH_OBJ{
    private:
		ros::NodeHandle _nh;
    
    public:
	//	SEARCH_OBJ();
		void input();
		void run();
        void move(float x_des, float y_des, float w_des);
		void controlLoop();
		bool ok = false;
		int marker_id_des=0;
		float x_in= 0.0;
		float y_in= 0.0;
		float w_in = 1.0;
		bool arrivo = false;

};

void SEARCH_OBJ::input() {

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

	ok = true;	
}

void SEARCH_OBJ::move(float x_des, float y_des, float w_des){

//tell the action client that we want to spin a thread by default
 MoveBaseClient ac("move_base", true);
//wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }
  
    move_base_msgs::MoveBaseGoal goal;
   
     //we'll send a goal to the robot to move 1 meter forward
     goal.target_pose.header.frame_id = "map";
     goal.target_pose.header.stamp = ros::Time::now();
   
     goal.target_pose.pose.position.x = x_des;
	 goal.target_pose.pose.position.y = y_des;
	 goal.target_pose.pose.orientation.w = w_des;
   
     ROS_INFO("Sending goal");
     ac.sendGoal(goal);
   
     ac.waitForResult();
   
     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       {  ROS_INFO("Hooray, the base moved 1 meter forward");
	   arrivo=true;
	   }
     else
       ROS_INFO("The base failed to move forward 1 meter for some reason");
	
   
}

void SEARCH_OBJ::controlLoop(){


    bool not_found = false;
    if(ok==true){
        move(x_in, y_in, w_in);
        if (arrivo==true)
            {ROS_INFO("Goal raggiunto fai rotazione");
            
            move(0.0, 3.0, 2.0);
            ROS_INFO("Rotazione fatta ");
            }
        else
            ROS_INFO("Goal non raggiunto");


    }
}
void SEARCH_OBJ::run() {
	//boost::thread input_t( &SEARCH_OBJ::input, this );
	//boost::thread move_t( &SEARCH_OBJ::move, this );
	boost::thread controlLoop_t( &SEARCH_OBJ::controlLoop, this);
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "search_obj"); //node name

	SEARCH_OBJ so;
	so.input();
	so.run();
//	aruco_msgs::Marker & marker_i = marker_msg_->markers.at;
//	if (marker_i.id==marker_id_des)
//	cout<<"trovato" <<endl;
	return 0;
}