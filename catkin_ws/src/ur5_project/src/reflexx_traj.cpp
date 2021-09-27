//  ---------------------- Doxygen info ----------------------
//! \file 01_RMLPositionSampleApplication.cpp
//!
//! \brief
//! Test application number 1 for the Reflexxes Motion Libraries
//! (basic position-based interface)
//!
//! \date March 2014
//!
//! \version 1.2.6
//!
//! \author Torsten Kroeger, <info@reflexxes.com> \n
//!
//! \copyright Copyright (C) 2014 Google, Inc.
//! \n
//! \n
//! <b>GNU Lesser General Public License</b>
//! \n
//! \n
//! This file is part of the Type II Reflexxes Motion Library.
//! \n\n
//! The Type II Reflexxes Motion Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU Lesser General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Type II Reflexxes Motion Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU Lesser General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU Lesser General Public License
//! along with the Type II Reflexxes Motion Library. If not, see
//! <http://www.gnu.org/licenses/>.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include "opencv2/opencv.hpp"


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.005
#define NUMBER_OF_DOFS                          6


//*************************************************************************
// Main function to run the process that contains the test application
//
// This function contains source code to get started with the Type II
// Reflexxes Motion Library. Only a minimum amount of functionality is
// contained in this program: a simple trajectory for a
// three-degree-of-freedom system is executed. This code snippet
// directly corresponds to the example trajectories shown in the
// documentation.
//*************************************************************************

trajectory_msgs::JointTrajectory plan;
bool plan_available = false;
bool rob_pos_received = false;
int number_of_points = 0;

geometry_msgs::Twist rob_pos;

double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
bool init_pos_received = false;
bool initial_point_reached = false; 

trajectory_msgs::JointTrajectory error;
bool error_received = false; 

void get_pos(const geometry_msgs::Twist & _data){
	rob_pos = _data;
	rob_pos_received = true;
	 if (!init_pos_received) {
        init_x = _data.linear.x;
        init_y = _data.linear.y;
        init_z = _data.linear.z;
        init_roll = _data.angular.x;
		if( init_roll>0){
			 init_roll =  init_roll - M_PI * 2 ;
		}
        init_pitch = _data.angular.y;
        init_yaw = _data.angular.z;
        init_pos_received = true;
    }
}


int main(int argc, char * argv[])
{

	ros::init(argc,argv,"reflexx_traj");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");

	bool external_plan = false;
	home.getParam("external_plan",external_plan);
	

	int loop_freq = 10;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	/*	TODO
			1. Publisher for sending geometry_msgs::Twist at node "/reftraj".
			2. Subscriber of the robot position "/robot/worldpos/" with the callback function get_pos.
	*/
	/*
	 * CODE HERE.
	*/    

    geometry_msgs::Twist ref;

    // ********************************************************************
    // Variable declarations and definitions

    int                         ResultValue                 =   0       ;

    ReflexxesAPI                *RML                        =   NULL    ;

    RMLPositionInputParameters  *IP                         =   NULL    ;

    RMLPositionOutputParameters *OP                         =   NULL    ;

    RMLPositionFlags            Flags                                   ;
    
    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
                                            ,   CYCLE_TIME_IN_SECONDS   );

    IP  =   new RMLPositionInputParameters(     NUMBER_OF_DOFS          );

    OP  =   new RMLPositionOutputParameters(    NUMBER_OF_DOFS          );

    // ********************************************************************
    // Set-up a timer 
    
    // ********************************************************************
    
    
    // ********************************************************************
    // Set-up the input parameters

    // In this test program, arbitrary values are chosen. If executed on a
    // real robot or mechanical system, the position is read and stored in
    // an RMLPositionInputParameters::CurrentPositionVector vector object.
    // For the very first motion after starting the controller, velocities
    // and acceleration are commonly set to zero. The desired target state
    // of motion and the motion constraints depend on the robot and the
    // current task/application.
    // The internal data structures make use of native C data types
    // (e.g., IP->CurrentPositionVector->VecData is a pointer to
    // an array of NUMBER_OF_DOFS double values), such that the Reflexxes
    // Library can be used in a universal way.


	// initialize the current position
	while (!init_pos_received) {
                loop_rate.sleep();
                ros::spinOnce();
    }
    IP -> CurrentPositionVector -> VecData[0] = init_x;
    IP -> CurrentPositionVector -> VecData[1] = init_y;
    IP -> CurrentPositionVector -> VecData[2] = init_z;
    IP -> CurrentPositionVector -> VecData[3] = init_roll;
    IP -> CurrentPositionVector -> VecData[4] = init_pitch;
    IP -> CurrentPositionVector -> VecData[5] = init_yaw;

	if(external_plan){
		
	}else{

    // initializing the solver for producing the trajectories with position, vel, acc and their constraints
		// You can use the following recommended values
		double vel_max = 0.1; 
		double acc_max = 0.25;
		double jer_max = 0.5;
		// TODO: Initlize the solver with current vel, current acc, max vel, max acc, max jerk, target pos, vel, acc
        // You can add small increments (0.02 m or 0.5 rad)to the initial positions to get a close and safe target position
        // initial positions are retrieved in: init_x, init_y, init_z, init_roll, init_pitch, init_yaw
		/*
		 * CODE HERE.
		*/

		//determine which Degrees of freedom should be calculated
		IP->SelectionVector->VecData            [0] =   true        ;
		IP->SelectionVector->VecData            [1] =   true        ;
		IP->SelectionVector->VecData            [2] =   true        ;
		IP->SelectionVector->VecData            [3] =   true        ;
		IP->SelectionVector->VecData            [4] =   true        ;
		IP->SelectionVector->VecData            [5] =   true        ;
	}

    // ********************************************************************
    // Starting the control loop
    while(ros::ok()){

		if (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED){
        // ****************************************************************
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************
		
		
		// std::cout << "updating values "<<std::endl;
        // Calling the Reflexxes OTG algorithm
        ResultValue =   RML->RMLPosition(       *IP
                                            ,   OP
                                            ,   Flags       );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }



        // ****************************************************************
        // Here, the new state of motion, that is
        //
        // - OP->NewPositionVector
        // - OP->NewVelocityVector
        // - OP->NewAccelerationVector
        //
        // can be used as input values for lower level controllers. In the
        // most simple case, a position controller in actuator space is
        // used, but the computed state can be applied to many other
        // controllers (e.g., Cartesian impedance controllers,
        // operational space controllers).
        // ****************************************************************

		// ***********Controller starts here ******************************
		// Add pd controller to the new position (expected position)
		// if(initial_point_reached && error_received){ // if the first point on the contour is reached and the error is received, add error terms to the x and y position
		// 	// P controller
		// 	OP->NewPositionVector->VecData         [0] = OP->NewPositionVector->VecData [0] - kp * pos_error_x; 
		// 	OP->NewPositionVector->VecData         [1] = OP->NewPositionVector->VecData [1] - kp * pos_error_y; 
		// 	std::cout <<"P controller working" << std::endl;

		// 	// // PD controller ( there is no sensor measuring the velocity, so maybe d controller is not applicable)
		// 	// OP->NewPositionVector->VecData         [0] = OP->NewPositionVector->VecData [0] - kp * pos_error.x - kd * vel_error.x; 
		// 	// OP->NewPositionVector->VecData         [1] = OP->NewPositionVector->VecData [1] - kp * pos_error.y - kd * vel_error.y; 
		// }
		

		// ***********Controller ends here ********************************


        // ****************************************************************
        // Feed the output values of the current control cycle back to
        // input values of the next control cycle

		

        *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
        *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
        *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
		
		
		ref.linear.x = IP->CurrentPositionVector->VecData[0];
		ref.linear.y = IP->CurrentPositionVector->VecData[1];
		ref.linear.z = IP->CurrentPositionVector->VecData[2];
		ref.angular.x = IP->CurrentPositionVector->VecData[3];
		ref.angular.y = IP->CurrentPositionVector->VecData[4];
		ref.angular.z = IP->CurrentPositionVector->VecData[5];

		//reflexxes_pub.publish(ref); // TODO: recover this line.
		
		}
		

		loop_rate.sleep();
		ros::spinOnce();
		//********************************************************************
		// Deleting the objects of the Reflexxes Motion Library end terminating the process
	}
	delete  RML         ;
	delete  IP          ;
	delete  OP          ;

	exit(EXIT_SUCCESS)  ;
}
