#include <ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <kdl/chain.hpp>
#include "Eigen/Core"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

using namespace std;
using namespace KDL;
// TODO: set move_to_target to false for retrieving current xyzrpy without moving the robot; or set move_to_target to true for executing desired motion. Please compile after making changes 
bool move_to_target = false;

/*	TODO: copy and paste your kdl chain definitions 
*/
KDL::Chain LWR() {
	KDL::Chain chain;
	// base
	Frame R, T;
	R = Frame(Rotation::RPY(0,0,0));
	T = Frame(Vector());
	Frame frame1 = Frame(T * R);
	chain.addSegment(Segment(Joint(Joint::None), frame1));
	// shoulder_pan_joint
	R = Frame();
	T = Frame();
	Frame frame2 = Frame(T * R);
	chain.addSegment(Segment(Joint(Joint::RotZ), frame2));
	// shoulder_lift_joint
	R = Frame();
	T = Frame();
	Frame frame3 = Frame(T * R);
	chain.addSegment(Segment(Joint(Joint::RotZ), frame3));
	// elbow joint
	R = Frame();
	T = Frame();
	Frame frame4 = Frame(T * R);
	chain.addSegment(Segment(Joint(Joint::RotZ), frame4));
	// wrist 1
	R = Frame();
	T = Frame();
	Frame frame5 = Frame(T * R);
	chain.addSegment(Segment(Joint(Joint::RotZ), frame5));
	// wrist 2
	R = Frame();
	T = Frame();
	Frame frame6 = Frame(T * R);
	chain.addSegment(Segment(Joint(Joint::RotZ), frame6));
	// wrist 3
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.0))));
	return chain;
}


/*	Callback for reading joint_states values.
*/
bool initialized = false;
bool joint_received = false;
sensor_msgs::JointState joints;
void get_joint_states(const sensor_msgs::JointState & data) {
	if (initialized) {
    joints.position[0] = data.position[2];
    joints.position[1] = data.position[1];
    joints.position[2] = data.position[0];
    joints.position[3] = data.position[3];
    joints.position[4] = data.position[4];
    joints.position[5] = data.position[5];
		joint_received = true;
	} else {
    for (int i = 0; i < 6; i++) {
  		joints.position.push_back(0.0);
    }
	}
	initialized = true;
}


/*	Initialize the joint positions with a non-zero value to be used in the solvers.
*/
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init) {
      for (int i = 0; i < _nj; ++i)
            _jointpositions(i) = _init;
}


/*	Initialize a joint command point.
*/
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init) {
      for (int i = 0; i < _nj; ++i)
            _pt.positions.push_back(_init);
}


/*	TODO: copy and paste eval_points() from manual.cpp
*/
void eval_points(trajectory_msgs::JointTrajectoryPoint & _point, KDL::JntArray & _jointpositions, int _nj) {
	/*
	 * CODE HERE.
	*/

}


/*	TODO: Complete get_ref() function.
		Call back function for subscriber ref_sub.
*/
bool ref_received = false;
geometry_msgs::Twist refl;
void get_ref(const geometry_msgs::Twist & data) {
	/*
	 * CODE HERE.
	*/

	ref_received = true;
}


int main(int argc, char * argv[]) {
	// Define the kinematic chain related.
	KDL::Chain chain = LWR();
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
	KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);
	KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, 1e-6);
	unsigned int nj = chain.getNrOfJoints();
	KDL::JntArray jointpositions = KDL::JntArray(nj);
	KDL::JntArray jointpositions_new = KDL::JntArray(nj);
	initialize_joints(jointpositions, nj, 0.2);
	geometry_msgs::Twist xyz;
	KDL::Frame cartpos;

	// Define the ros node related.
  ros::init(argc, argv, "reflexxes");
  ros::NodeHandle nh_;
  int loop_freq = 10;
  float dt = (float) 1 / loop_freq;
  ros::Rate loop_rate(loop_freq);
	tf::TransformBroadcaster br;
	tf::Transform tool_in_base_link;
  ros::Publisher cmd_pub = nh_.advertise < trajectory_msgs::JointTrajectory > ("/scaled_pos_joint_traj_controller/command", 10);
	ros::Subscriber jointStates_sub = nh_.subscribe("/joint_states", 10, get_joint_states);
	ros::Publisher xyzrpy_pub = nh_.advertise < geometry_msgs::Twist > ("/robot/worldpos", 10);
	// TODO: Define a subscriber named ref_sub to subscribe from rostopic /reftraj for reading the reference trajectories from the reflexx_traj.cpp node.
	/*
	 * CODE HERE.
	*/

  // Define trajectory point.
  trajectory_msgs::JointTrajectoryPoint pt;
  initialize_points(pt, nj, 0.0);

	// TODO: copy and paste joint names definitions
	trajectory_msgs::JointTrajectory joint_cmd;
	/*
	 * CODE HERE.
	*/

	while(!joint_received){
		ros::spinOnce();
		loop_rate.sleep();
	}

	// TODO: copy and paste jointpositions definition from ik.cpp.
	/*
	 * CODE HERE.
	*/

	eval_points(pt, jointpositions, nj);
	pt.time_from_start = ros::Duration(5.0);
	joint_cmd.header.stamp = ros::Time::now();
	joint_cmd.points.push_back(pt);

  while (ros::ok()) {
		if (initialized) {
			bool kinematics_status;
			double roll, pitch, yaw, x, y, z;
			for (int i = 0; i < 6; i++) {
				jointpositions(i) = joints.position[i];
			}

			if (ref_received) {
			// TODO: Update jointpositions_new using reflexxe_traj.cpp calculated values and inverse kinematics solver.
			/*	
			 * CODE HERE.

			*/
			eval_points(pt, jointpositions_new, nj);
			pt.time_from_start = ros::Duration(dt);
			joint_cmd.points[0] = pt;
			joint_cmd.header.stamp = ros::Time::now();
			if(move_to_target){
					cmd_pub.publish(joint_cmd);
				}
			}
			// Debugging code. echo "/robot/worldpos" in a terminal or watch "tool_from_kdl" in rviz. 
			kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
			if (kinematics_status >= 0) {
				xyz.linear.x = cartpos.p[0];
				xyz.linear.y = cartpos.p[1];
				xyz.linear.z = cartpos.p[2];
				cartpos.M.GetRPY(roll, pitch, yaw);
				xyz.angular.x = roll;
				xyz.angular.y = pitch;
				xyz.angular.z = yaw;
				tool_in_base_link.setOrigin( tf::Vector3(cartpos.p[0], cartpos.p[1], cartpos.p[2]) );
				tf::Quaternion tool_orientation;
				tool_orientation.setRPY(roll, pitch,  yaw); 
				tool_in_base_link.setRotation( tool_orientation );
				br.sendTransform(tf::StampedTransform(tool_in_base_link, ros::Time::now(), "base_link", "tool_from_kdl"));
			}
			joint_cmd.header.stamp = ros::Time::now();
			if (move_to_target) {
				cmd_pub.publish(joint_cmd);
			}
			xyzrpy_pub.publish(xyz);
		}
		loop_rate.sleep();
		ros::spinOnce();     
	} 
	return 0;
}
