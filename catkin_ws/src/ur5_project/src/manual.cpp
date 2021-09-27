#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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

/*	Initialize a joint command point.
*/
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init) {
      for (int i = 0; i < _nj; ++i)
            _pt.positions.push_back(_init);
}


/*	TODO: Complete eval_points() function.
		Loads the joint space points (_jointpositions) to be sent as a command to the robot (_point). Make sure all joint values falling into UR5 robot joint limits, which is from -pi to pi. Note: use M_PI for pi.
*/
bool initialized = false;
void eval_points(trajectory_msgs::JointTrajectoryPoint & _point, KDL::JntArray & _jointpositions, int _nj) {
	for (int i = 0; i < _nj; ++i){
		while (_jointpositions(i) > M_PI){
			_jointpositions(i) -= 2*M_PI;
		}
		while (_jointpositions(i) < -M_PI){
			_jointpositions(i) += 2*M_PI;
		}
		_point.positions[i] = _jointpositions(i);
	}
	initialized = true;
}


int main(int argc, char * argv[]) {
	ros::init(argc, argv, "manual");
	ros::NodeHandle nh_;
	int loop_freq = 10;
	float dt = (float) 1 / loop_freq;
	ros::Rate loop_rate(loop_freq);
	ros::Publisher cmd_pub = nh_.advertise < trajectory_msgs::JointTrajectory > ("/scaled_pos_joint_traj_controller/command", 10);

	// Define trajectory point.
	trajectory_msgs::JointTrajectoryPoint pt;
	unsigned int nj = 6;
	initialize_points(pt, nj, 0.0);

	// TODO: Define the ur5 joint names in joint_cmd variable.
	trajectory_msgs::JointTrajectory joint_cmd;
	joint_cmd.joint_names.push_back("elbow_joint");
	joint_cmd.joint_names.push_back("shoulder_lift_joint");
	joint_cmd.joint_names.push_back("shoulder_pan_joint");
	joint_cmd.joint_names.push_back("wrist_1_joint");
	joint_cmd.joint_names.push_back("wrist_2_joint");
	joint_cmd.joint_names.push_back("wrist_3_joint");

	// TODO: Finish manual_joint_cmd by defining each joint value. Please make small changes to the current joint values (0.5 rad)
  	KDL::JntArray manual_joint_cmd = KDL::JntArray(nj);
	double delta = -10.0/180.0 * M_PI - 0.1;
	manual_joint_cmd(2) =  delta;
	manual_joint_cmd(1) =  delta;
	manual_joint_cmd(0) =  delta;
	manual_joint_cmd(3) =  delta;
	manual_joint_cmd(4) =  delta;
	manual_joint_cmd(5) =  delta;


	eval_points(pt, manual_joint_cmd, nj);
	pt.time_from_start = ros::Duration(5.0);
	joint_cmd.header.stamp = ros::Time::now();
	joint_cmd.points.push_back(pt);

  while (ros::ok()) {
		if (initialized) {
			joint_cmd.header.stamp = ros::Time::now();
			cmd_pub.publish(joint_cmd);
		}
		loop_rate.sleep();
		ros::spinOnce();     
	} 
	return 0;
}
