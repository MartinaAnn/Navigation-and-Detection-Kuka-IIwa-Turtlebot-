#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>
#include <aruco/aruco.h>
#include <aruco_msgs/Marker.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

using namespace std;


class KUKA_INVKIN {

		private:
		ros::NodeHandle _nh;
		ros::Subscriber _js_sub;
		ros::Publisher _cartpose_pub;
		ros::Publisher _cmd_pub[7];
		ros::Subscriber _marker_sub;
		ros::Subscriber _ee_sub;
		sensor_msgs::JointState js;
		
		KDL::Tree iiwa_tree;
		KDL::ChainFkSolverPos_recursive *_fksolver; //Forward position solver	
		KDL::ChainIkSolverVel_pinv *_ik_solver_vel;   	//Inverse velocity solver
		KDL::ChainIkSolverPos_NR *_ik_solver_pos;
		KDL::Chain _k_chain;
		KDL::JntArray *_q_in;
		KDL::Frame _p_out;
		
			geometry_msgs::Pose cpose;
		
		
		bool _first_js;
		bool _first_fk;
		geometry_msgs::PoseStamped _marker_pose; //possibile che il controllo possa essere fatto con questo
		geometry_msgs::Pose _eef_pose;
		const geometry_msgs::PoseStamped::ConstPtr markerpose;
	
		std::vector<geometry_msgs::PoseStamped> check_size_marker;
		bool seemarker;
		
		KDL::Frame goal;
		
		
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
		void get_dirkin();
		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();
		void goto_initial_position( float dp[7] );
		void approaching(KDL::Frame goal);
		void markerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &markerpose);
		
};


bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("/kuka_iiwa/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::string base_link = "iiwa_link_0";
	std::string tip_link  = "iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;
	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );

	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	return true;
}


KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

	cout << "Joints and segments: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
 
	_cartpose_pub = _nh.advertise<geometry_msgs::Pose>("/kuka_iiwa/eef_pose", 0);
	_js_sub = _nh.subscribe("/kuka_iiwa/joint_states", 0, &KUKA_INVKIN::joint_states_cb, this);

	
	_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/kuka_iiwa/joint1_position_controller/command", 0);
	_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/kuka_iiwa/joint2_position_controller/command", 0);
	_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/kuka_iiwa/joint3_position_controller/command", 0);
	_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/kuka_iiwa/joint4_position_controller/command", 0);
	_cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/kuka_iiwa/joint5_position_controller/command", 0);
	_cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/kuka_iiwa/joint6_position_controller/command", 0);
	_cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/kuka_iiwa/joint7_position_controller/command", 0);

	_first_js = false;
	_first_fk = false;
	seemarker=false;
	
	_marker_sub = _nh.subscribe("simple_single/pose",10, &KUKA_INVKIN::markerPoseCallback,this);

}




void KUKA_INVKIN::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) 
		_q_in->data[i] = js.position[i];

	_first_js = true;
}


void KUKA_INVKIN::goto_initial_position( float dp[7] ) {
	
	ros::Rate r(10);
	float min_e = 1000;

	std_msgs::Float64 cmd[7];

	float max_e = 1000;
	while( max_e > 0.002 ) {
 		max_e = -1000;
		for(int i=0; i<7; i++) {
 			cmd[i].data = dp[i];
			_cmd_pub[i].publish (cmd[i]);
			float e = fabs( cmd[i].data - _q_in->data[i] );
			max_e = ( e > max_e ) ? e : max_e;
			//cout << fabs( cmd[i].data - _q_in->data[i] ) << endl;
		}
		r.sleep();
	}

	sleep(2);
}



void KUKA_INVKIN::get_dirkin() {

	ros::Rate r(50);


	KDL::JntArray q_curr(_k_chain.getNrOfJoints());


	while( !_first_js ) usleep(0.1);


	while(ros::ok()) {


		_fksolver->JntToCart(*_q_in, _p_out);

		cpose.position.x = _p_out.p.x();
		cpose.position.y = _p_out.p.y();
		cpose.position.z = _p_out.p.z();

		double qx, qy, qz, qw;
		_p_out.M.GetQuaternion( qx, qy, qz, qw);
		cpose.orientation.w = qw;
		cpose.orientation.x = qx;
		cpose.orientation.y = qy;
		cpose.orientation.x = qz;

		_cartpose_pub.publish( cpose );		
		_first_fk = true;
	
		r.sleep();
	}
}


void KUKA_INVKIN::approaching(KDL::Frame goal){


KDL::JntArray q_out(_k_chain.getNrOfJoints());
_fksolver->JntToCart(*_q_in, _p_out);
goal.p.data[0]=_p_out.p.x();
goal.p.data[1]=_p_out.p.y();
goal.p.data[2]=_p_out.p.z();

std_msgs::Float64 cmd[7];
while(ros::ok){
if( _ik_solver_pos->CartToJnt(*_q_in, goal, q_out) != KDL::SolverI::E_NOERROR ) 
		cout << "failing in ik!" << endl;

	for(int i=0; i<7; i++) {
		cmd[i].data = q_out.data[i]; 
	}
	for(int i=1; i<7; i++) {
		_cmd_pub[i].publish (cmd[i]);
	}
	}

cout<< "cpose.position.x" <<cpose.position.x<<endl;
}

void KUKA_INVKIN::ctrl_loop() {

	std_msgs::Float64 d;
	
	seemarker=false;
	while( !_first_fk ) usleep(0.1);
	
	float i_cmd[7];
	i_cmd[0] = -1.57;
	i_cmd[1] = i_cmd[4] = i_cmd[2] = i_cmd[6] = 0.0;
	i_cmd[3] = -1.57;
	i_cmd[5] = 1.57;
	goto_initial_position( i_cmd );

	//ros::Rate r(50);
	
	KDL::Frame F_dest; //ha un campo posizione p e uno orientamento M
	KDL::JntArray q_out(_k_chain.getNrOfJoints());


	F_dest.p.data[0] = _p_out.p.x();
	F_dest.p.data[1] = _p_out.p.y();
	F_dest.p.data[2] = _p_out.p.z();
	

	for(int i=0; i<9; i++ )
		F_dest.M.data[i] = _p_out.M.data[i];

	std_msgs::Float64 cmd[7];
	usleep(1000000);
	
	if( _ik_solver_pos->CartToJnt(*_q_in, F_dest, q_out) != KDL::SolverI::E_NOERROR ) 
		cout << "failing in ik!" << endl;

	for(int i=0; i<7; i++) {
		cmd[i].data = q_out.data[i]; 
	}
	for(int i=1; i<7; i++) {
		_cmd_pub[i].publish (cmd[i]);
	}
	cout<<"Kuka took the object, now it will place it."<<endl;
	float step=0.2;
	seemarker=true;
	while(check_size_marker.size()==0){
		cmd[0].data=cmd[0].data+step;
		_cmd_pub[0].publish (cmd[0]);
		usleep(100000);
	}
	seemarker=false;
	if(check_size_marker.size()!=0){
	_first_js=false;
		approaching(F_dest);
	
	}
}


void KUKA_INVKIN::markerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &markerpose){
	if(seemarker){
	seemarker=false;
	check_size_marker.push_back(*markerpose);
	//cout<<"size marker array: " << check_size_marker.size() <<endl;
	_marker_pose.pose= markerpose -> pose;
	cout<<"marker array: " << _marker_pose.pose.position <<endl;
	}
}



void KUKA_INVKIN::run() {
	boost::thread markerPoseCallback_t(&KUKA_INVKIN::markerPoseCallback, this, markerpose);
	boost::thread get_dirkin_t( &KUKA_INVKIN::get_dirkin, this);
	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	

}




int main(int argc, char** argv) {

	ros::init(argc, argv, "kuka_iiwa_kdl");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
