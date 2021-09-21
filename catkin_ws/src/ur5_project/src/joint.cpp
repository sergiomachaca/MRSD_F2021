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
      //TODO create the KDL chain with the ur5 calibration file 
      KDL::Chain chain;
      // base -> shoulder
      Frame R, T;
      T = Frame(Vector(0, 0, 0.08929437215831398));
      R = Frame(Rotation::RPY(0,0,1.194158776760344e-05));
      Frame frame1 = Frame(T * R);
      chain.addSegment(Segment(Joint(Joint::None), frame1));
      // shoulder -> upper_arm
      T = Frame(Vector(0.0001059934666038868, 0, 0));
      R = Frame(Rotation::RPY(1.570288659480724, 0, -4.360963378713104e-05));
      Frame frame2 = Frame(T * R);
      chain.addSegment(Segment(Joint(Joint::RotZ), frame2));
      // upper_arm -> forearm
      T = Frame(Vector(-0.4248744108132448, 0, 0));
      R = Frame(Rotation::RPY(3.140689141970196,3.141029629875079,3.141577753679916));
      Frame frame3 = Frame(T * R);
      chain.addSegment(Segment(Joint(Joint::RotZ), frame3));
      // forearm -> wrist 1
      T = Frame(Vector(-0.3921965391248423, -0.001215901650067357, 0.1108697089194659));
      R = Frame(Rotation::RPY(0.01096650219786626,0.0007089237936044265,-6.461093627887461e-05));
      Frame frame4 = Frame(T * R);
      chain.addSegment(Segment(Joint(Joint::RotZ), frame4));
      // wrist 1 -> wrist 2
      T = Frame(Vector(-4.432612574180066e-05,-0.09485818230946315,3.804656520711195e-05));
      R = Frame(Rotation::RPY(1.570395237903663,0,1.346641505478031e-05));
      Frame frame5 = Frame(T * R);
      chain.addSegment(Segment(Joint(Joint::RotZ), frame5));
      // wrist 2 -> wrist 3
      T = Frame(Vector(5.523513679646697e-05,0.08273360798729773,-7.130081408339577e-06));
      R = Frame(Rotation::RPY(1.570710145597628,3.141592653589793,-3.141579371363952));
      Frame frame6 = Frame(T * R);
      chain.addSegment(Segment(Joint(Joint::RotZ), frame6));
      // wrist 3 -> end_effector
      chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.0))));

      return chain;
}

sensor_msgs::JointState joints;
bool initialized = false;
bool joint_received = false;

// Create the frame that will contain the results
      KDL::Frame cartpos;



//callback for reading joint_states values
void get_joint_states(const sensor_msgs::JointState & data) {

     //TODO finish the call back function get_joint_states
     if (initialized){
           joints.position[0] = data.position[2];
           joints.position[1] = data.position[1];
           joints.position[2] = data.position[0];
           joints.position[3] = data.position[3];
           joints.position[4] = data.position[4];
           joints.position[5] = data.position[5];
     }
     else {
           for (int i = 0; i<6; ++i){
                  joints.position.push_back(0.0);
           }
           initialized = true;
     }
}


// initialize the joint positions with a non-zero value to be used in the solvers
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init) {
      for (int i = 0; i < _nj; ++i)
            _jointpositions(i) = _init;
}



int main(int argc, char * argv[]) {
      // define the kinematic chain
      KDL::Chain chain = LWR();
      // define the forward kinematic solver via the defined chain
      KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

      // define the inverse kinematics solver
      KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain); //Inverse velocity solver
      KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, 1e-4); //Maximum 100 iterations, stop at accuracy 1e-6

      std::cout << "KDL built successfully" << std::endl;

      // get the number of joints from the chain
      unsigned int nj = chain.getNrOfJoints();
      KDL::JntArray jointpositions = KDL::JntArray(nj);
      
      // std::cout << nj << std::endl;


      // Create the frame that will contain the results
      KDL::Frame cartpos;

      // define the ros node
      ros::init(argc, argv, "joint");
      ros::NodeHandle nh_;

      // setting up the loop frequency
      int loop_freq = 10;
      float dt = (float) 1 / loop_freq;
      ros::Rate loop_rate(loop_freq);

      // broadcast tf transform
      tf::TransformBroadcaster br;
      tf::Transform transform;

      
      initialize_joints(jointpositions, nj, 0.2);

      //for debugging: Calculate forward position kinematics
      bool kinematics_status;
      double roll, pitch, yaw, x, y, z;
      kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
      if(kinematics_status>=0){
            cartpos.M.GetRPY(roll,pitch, yaw);
      }


      // //add subscriber -----------------------------------------------------
      ros::Subscriber jointStates_sub = nh_.subscribe("/joint_states", 10, get_joint_states);
      ros::Publisher  xyzrpy_pub      = nh_.advertise<geometry_msgs::Twist>("robot/worldpos", 10);
      geometry_msgs::Twist xyzrpy;
      initialize_joints(jointpositions, nj, 0.2);
     

      while (ros::ok()) {
            if (initialized) {

                  // update the joint positions with the most recent readings from the joints
                  for (int i = 0; i < 6; i++) {
                        jointpositions(i) = joints.position[i];
                  }

                  //std::cout<<"joint1 "<<jointpositions(0)<<"joint2 "<<jointpositions(1)<<"joint3 "<<jointpositions(2)<<std::endl;

                  kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
                  if(kinematics_status>=0){
                        cartpos.M.GetRPY(roll,pitch, yaw);
                        //cartpos.p.x(x);
                        //cartpos.p.y(y);
                        //cartpos.p.z(z);
                  }

                  //std::cout<<"x "<<cartpos.p[0]<<" y "<<cartpos.p[1]<<" z "<<cartpos.p[2]<<std::endl;

                  xyzrpy.linear.x = x;
                  xyzrpy.linear.y = y;
                  xyzrpy.linear.z = z;
                  xyzrpy.angular.x = roll;
                  xyzrpy.angular.y = pitch;
                  xyzrpy.angular.z = yaw; 

                  xyzrpy_pub.publish(xyzrpy);

                  //-----------------TF transform----------------------------
                  transform.setOrigin(tf::Vector3(cartpos.p[0], cartpos.p[1], cartpos.p[2]));
                  tf::Quaternion q;
                  q.setRPY(roll, pitch, yaw);
                  transform.setRotation(q);
                  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "TCP_to_Base"));
                  // std::cout<<"hello1"<<std::endl;
                  //------------------------------------------------------------------
            }
            // std::cout << "hello" << std::endl;
            loop_rate.sleep();
            ros::spinOnce();

           
      }

      
      return 0;
}