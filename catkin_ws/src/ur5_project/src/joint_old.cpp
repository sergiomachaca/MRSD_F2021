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


using namespace KDL;
using namespace std;

KDL::Chain LWR() {
      KDL::Chain chain;
      //base
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
        KDL::Frame::DH_Craig1989(0,0,0.33989,0)));
  //joint 1
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));
  //joint 2
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0.40011,0)));
  //joint 3
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0,0)));
  //joint 4
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0.40003,0)));
  //joint 5
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));
      return chain;
}


int main(int argc, char * argv[]) {
      // define the kinematic chain
      KDL::Chain chain = LWR();
      // define the forward kinematic solver via the defined chain
      KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

      // define the inverse kinematics solver
      KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain); //Inverse velocity solver
      KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, 1e-4); //Maximum 100 iterations, stop at accuracy 1e-6

      // get the number of joints from the chain
      unsigned int nj = chain.getNrOfJoints();

      ros::init(argc, argv, "joint");
      ros::NodeHandle nh_;

      // setting up the loop frequency
      int loop_freq = 10;
      float dt = (float) 1 / loop_freq;
      ros::Rate loop_rate(loop_freq);

      std::cout << "KDL built successfully" << std::endl;

      // robot position publisher, will be subscribed in reflexx_traj
      ros::Publisher xyzrpy_pub = nh_.advertise < geometry_msgs::Twist > ("/robot/worldpos", 10);
      geometry_msgs::Twist xyzrpy;

      double roll, pitch, yaw, x, y, z;

      while (ros::ok()) {
            // TODO: finish xyzrpy_pub publisher using kdl forward kinematics

            loop_rate.sleep();
            ros::spinOnce();

      }


      return 0;
}
