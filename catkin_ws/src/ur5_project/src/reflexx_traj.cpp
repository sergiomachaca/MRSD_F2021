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

double vel_max = 0.1; 
double acc_max = 0.25;
double jer_max = 0.5;


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

void get_plan(const trajectory_msgs::JointTrajectory & _data){
	plan = _data;
	// std::cout <<"plan points = " << plan.points[0] << std::endl;
	plan_available = true;
	number_of_points = plan.points.size();
}

void initialize_plan(RMLPositionInputParameters  *_IP){
		

		_IP->CurrentVelocityVector->VecData      [0] =    0     ;
		_IP->CurrentVelocityVector->VecData      [1] =    0      ;
		_IP->CurrentVelocityVector->VecData      [2] =    0      ;
		_IP->CurrentVelocityVector->VecData      [3] =    0      ;
		_IP->CurrentVelocityVector->VecData      [4] =    0      ;
		_IP->CurrentVelocityVector->VecData      [5] =    0      ;

		_IP->CurrentAccelerationVector->VecData  [0] =    0      ;
		_IP->CurrentAccelerationVector->VecData  [1] =    0      ;
		_IP->CurrentAccelerationVector->VecData  [2] =    0      ;
		_IP->CurrentAccelerationVector->VecData  [3] =    0      ;
		_IP->CurrentAccelerationVector->VecData  [4] =    0      ;
		_IP->CurrentAccelerationVector->VecData  [5] =    0      ;


		_IP->MaxVelocityVector->VecData          [0] =    vel_max      ;
		_IP->MaxVelocityVector->VecData          [1] =    vel_max      ;
		_IP->MaxVelocityVector->VecData          [2] =    vel_max      ;
		_IP->MaxVelocityVector->VecData          [3] =    vel_max      ;
		_IP->MaxVelocityVector->VecData          [4] =    vel_max      ;
		_IP->MaxVelocityVector->VecData          [5] =    vel_max      ;


		_IP->MaxAccelerationVector->VecData      [0] =    acc_max      ;
		_IP->MaxAccelerationVector->VecData      [1] =    acc_max      ;
		_IP->MaxAccelerationVector->VecData      [2] =    acc_max      ;
		_IP->MaxAccelerationVector->VecData      [3] =    acc_max      ;
		_IP->MaxAccelerationVector->VecData      [4] =    acc_max      ;
		_IP->MaxAccelerationVector->VecData      [5] =    acc_max      ;



		_IP->MaxJerkVector->VecData              [0] =    jer_max      ;
		_IP->MaxJerkVector->VecData              [1] =    jer_max      ;
		_IP->MaxJerkVector->VecData              [2] =    jer_max      ;
		_IP->MaxJerkVector->VecData              [3] =    jer_max      ;
		_IP->MaxJerkVector->VecData              [4] =    jer_max      ;
		_IP->MaxJerkVector->VecData              [5] =    jer_max      ;

		//setting the target velcoity and positions
		//TODO: setup the target positions vector and velocity vectors in terms of 'plan' *********************
		_IP->TargetPositionVector->VecData       [0] =   plan.points[0].positions[0];
		_IP->TargetPositionVector->VecData       [1] =   plan.points[0].positions[1];
		_IP->TargetPositionVector->VecData       [2] =   plan.points[0].positions[2];
		_IP->TargetPositionVector->VecData       [3] =   plan.points[0].positions[3];
		_IP->TargetPositionVector->VecData       [4] =   plan.points[0].positions[4];
		_IP->TargetPositionVector->VecData       [5] =   plan.points[0].positions[5];

		_IP->TargetVelocityVector->VecData       [0] =   plan.points[0].velocities[0];
		_IP->TargetVelocityVector->VecData       [1] =   plan.points[0].velocities[1];
		_IP->TargetVelocityVector->VecData       [2] =   plan.points[0].velocities[2];
		_IP->TargetVelocityVector->VecData       [3] =   plan.points[0].velocities[3];
		_IP->TargetVelocityVector->VecData       [4] =   plan.points[0].velocities[4];
		_IP->TargetVelocityVector->VecData       [5] =   plan.points[0].velocities[5];
		// ****************************************************************************************************

		//determine which Degrees of freedom should be calculated
		_IP->SelectionVector->VecData            [0] =   true        ;
		_IP->SelectionVector->VecData            [1] =   true        ;
		_IP->SelectionVector->VecData            [2] =   true        ;
		_IP->SelectionVector->VecData            [3] =   true        ;
		_IP->SelectionVector->VecData            [4] =   true        ;
		_IP->SelectionVector->VecData            [5] =   true        ;

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

	
    ros::Publisher reflexxes_pub = nh_.advertise<geometry_msgs::Twist>("/reftraj",1);
    ros::Subscriber pos_sub = nh_.subscribe("/robot/worldpos/" ,1, get_pos);
	ros::Subscriber plan_sub = nh_.subscribe("/plan",1,get_plan);

    

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
		while (!plan_available){
			loop_rate.sleep();
			ros::spinOnce();
		}
		initialize_plan(IP);
	}else{

    // initializing the solver for porducing the trajectories with position, vel, acc and their constraints
		
		// You can use the following recommended values
		double vel_max = 0.1; 
		double acc_max = 0.25;
		double jer_max = 0.5;
		
		
        IP->CurrentVelocityVector->VecData      [0] =    0.0      ;
		IP->CurrentVelocityVector->VecData      [1] =    0.0      ;
		IP->CurrentVelocityVector->VecData      [2] =    0.0      ;
		IP->CurrentVelocityVector->VecData      [3] =    0.0      ;
		IP->CurrentVelocityVector->VecData      [4] =    0.0      ;
		IP->CurrentVelocityVector->VecData      [5] =    0.0      ;

		IP->CurrentAccelerationVector->VecData  [0] =    0.0      ;
		IP->CurrentAccelerationVector->VecData  [1] =    0.0      ;
		IP->CurrentAccelerationVector->VecData  [2] =    0.0      ;
		IP->CurrentAccelerationVector->VecData  [3] =    0.0      ;
		IP->CurrentAccelerationVector->VecData  [4] =    0.0      ;
		IP->CurrentAccelerationVector->VecData  [5] =    0.0      ;


		IP->MaxVelocityVector->VecData          [0] =    vel_max      ;
		IP->MaxVelocityVector->VecData          [1] =    vel_max      ;
		IP->MaxVelocityVector->VecData          [2] =    vel_max      ;
		IP->MaxVelocityVector->VecData          [3] =    vel_max      ;
		IP->MaxVelocityVector->VecData          [4] =    vel_max      ;
		IP->MaxVelocityVector->VecData          [5] =    vel_max      ;


		IP->MaxAccelerationVector->VecData      [0] =    acc_max      ;
		IP->MaxAccelerationVector->VecData      [1] =    acc_max      ;
		IP->MaxAccelerationVector->VecData      [2] =    acc_max      ;
		IP->MaxAccelerationVector->VecData      [3] =    acc_max      ;
		IP->MaxAccelerationVector->VecData      [4] =    acc_max      ;
		IP->MaxAccelerationVector->VecData      [5] =    acc_max      ;



		IP->MaxJerkVector->VecData              [0] =    jer_max      ;
		IP->MaxJerkVector->VecData              [1] =    jer_max      ;
		IP->MaxJerkVector->VecData              [2] =    jer_max      ;
		IP->MaxJerkVector->VecData              [3] =    jer_max      ;
		IP->MaxJerkVector->VecData              [4] =    jer_max      ;
		IP->MaxJerkVector->VecData              [5] =    jer_max      ;

		//setting the target velcoity and positions
		IP->TargetPositionVector->VecData       [0] =    init_x + 0.02   ;
		IP->TargetPositionVector->VecData       [1] =   init_y     ;
		IP->TargetPositionVector->VecData       [2] =    init_z     ;
		IP->TargetPositionVector->VecData       [3] =   init_roll     ;
		IP->TargetPositionVector->VecData       [4] =   init_pitch   ;
		IP->TargetPositionVector->VecData       [5] =    init_yaw    ;


		IP->TargetVelocityVector->VecData       [0] =    0.0       ;
		IP->TargetVelocityVector->VecData       [1] =    0.0       ;
		IP->TargetVelocityVector->VecData       [2] =    0.0       ;
		IP->TargetVelocityVector->VecData       [3] =    0.0       ;
		IP->TargetVelocityVector->VecData       [4] =    0.0       ;
		IP->TargetVelocityVector->VecData       [5] =    0.0       ;

		//determine which Degrees of freedom should be calculated
		IP->SelectionVector->VecData            [0] =   true        ;
		IP->SelectionVector->VecData            [1] =   true        ;
		IP->SelectionVector->VecData            [2] =   true        ;
		IP->SelectionVector->VecData            [3] =   true        ;
		IP->SelectionVector->VecData            [4] =   true        ;
		IP->SelectionVector->VecData            [5] =   true        ;

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
	int ctr = 1; // counter for the plan 
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

		if (ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED && external_plan){
			//setting the target velcoity and positions
			
			int next_wp;
			next_wp = ctr % number_of_points;
			// TODO make setup the target position vector in terms of 'plan' 
			/* 
			*CODE HERE
			*/
			
			IP->TargetPositionVector->VecData       [0] =   plan.points[next_wp].positions[0] ;
			IP->TargetPositionVector->VecData       [1] =   plan.points[next_wp].positions[1] ;
			IP->TargetPositionVector->VecData       [2] =   plan.points[next_wp].positions[2] ;
			IP->TargetPositionVector->VecData       [3] =   plan.points[next_wp].positions[3] ;
			IP->TargetPositionVector->VecData       [4] =   plan.points[next_wp].positions[4] ;
			IP->TargetPositionVector->VecData       [5] =   plan.points[next_wp].positions[5] ;


  			if (rob_pos_received){
				IP->CurrentPositionVector->VecData[0] = rob_pos.linear.x;
				IP->CurrentPositionVector->VecData[1] = rob_pos.linear.y;
				IP->CurrentPositionVector->VecData[2] = rob_pos.linear.z;
				IP->CurrentPositionVector->VecData[3] = rob_pos.angular.x;
				IP->CurrentPositionVector->VecData[4] = rob_pos.angular.y;
				IP->CurrentPositionVector->VecData[5] = rob_pos.angular.z;
			}
			
			ctr ++;

			ResultValue =   RML->RMLPosition(       *IP
				                                ,   OP
				                                ,   Flags       );
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

		
		reflexxes_pub.publish(ref);
		
	    }
		

		loop_rate.sleep();
		ros::spinOnce();
		

	    // ********************************************************************
	    // Deleting the objects of the Reflexxes Motion Library end terminating
	    // the process
    }

    delete  RML         ;
    delete  IP          ;
    delete  OP          ;

    exit(EXIT_SUCCESS)  ;
}