/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Sariah Mghames. */


// C++ standard headers
#include <iostream>
#include <vector>
#include <exception>
#include <string>
//#include <iomanip>

// Boost headers
#include <boost/shared_ptr.hpp>


// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>//the controller exposes this interface in the follow_joint_trajectory namespace of the controller
#include <ros/topic.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

using namespace std ; 
// variables definition
vector<double> Pj0, Pj1, Pj2;
const int nb_joint = 3 ;
const int wpts = 18 ;
int lp = 18 ;
int lj1 = 2*lp ;
int lj2 = 3*lp ;
double T = 1.0 ;


//vector<double> Pj0(wpts), Pj1(wpts), Pj2(wpts), Vj0(wpts), Vj1(wpts), Vj2(wpts) ;
//double P, V ;

// A function that linearly divides a given interval into a given number of
  // points and assigns them to a vector

vector<double> linspace(double a, double b, int num)
{
  // create a vector of length num
  vector<double> v(num);

  // now assign the values to the vector
  for (int i = 0; i < num; i++){
    v.push_back(a + i * ( (b - a) / num ));
     }
  return v;
}


// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr; // arm_control_client_Ptr is a type and is a shared ptr btw elements/joints each of type arm_control_client
control_msgs::FollowJointTrajectoryGoal arm_goal ;



// Create a ROS action client to move GRASberry's arm
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/scara_arm/arm_traj_controller/follow_joint_trajectory") ); // action client on topic XX ... (/scara_arm/arm_traj_controller of type position_controllers/JointTrajectoryController or effort_controllers/JointTrajectoryController or velocity_controllers/JointTrajectoryController

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}


// Generates a simple trajectory with two waypoints to move GRASberry's arm 
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  ROS_INFO("Starting to store joint names ...");
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("/Plink1_plateM");
  goal.trajectory.joint_names.push_back("/plate_R1link2M");
  goal.trajectory.joint_names.push_back("/R1link2_R2link3M");
  

  // load waypoints in this goal trajectory (the npz for now is taken from the script: traj_control_reach_tryVel.py)

  //J = np.squeeze(Q1)
  vector<double> dt = linspace(0, T, lp) ;
  ROS_INFO("Get time vector ...");
  for(int i=0; i<dt.size(); ++i)
    cout << dt[i] << ' ';
    
  goal.trajectory.points.resize(lp);
 
  // First trajectory point
  // Positions
  for (int index = 0; index < lp; ++index) {
    ROS_INFO("Starting loop ...");
    
    //goal.trajectory.points[index].velocities.resize(nb_joint);

    ROS_INFO("resized done ...");
    //ROS_INFO("P0 %d = %f",index,P0.data[index]);
    goal.trajectory.points[index].positions.resize(nb_joint);
  	goal.trajectory.points[index].positions[0] = Pj0[index];
    goal.trajectory.points[index].positions[1] = Pj1[index];
    goal.trajectory.points[index].positions[2] = Pj2[index];
    ROS_INFO("stored pos ...%i", index);
  
    // To be reached 2 second after starting along the trajectory
    if (index == 0)
      goal.trajectory.points[index].time_from_start = ros::Duration(index+5);
    if (index >=1 and index <=6)
      goal.trajectory.points[index].time_from_start = ros::Duration(index+10);
    if (index > 6 and index <13)
      goal.trajectory.points[index].time_from_start = ros::Duration(index+20);
    if (index >= 13)
      goal.trajectory.points[index].time_from_start = ros::Duration(index+20);

}
}


void prompCallback0(const std_msgs::Float64MultiArray::ConstPtr& msg0){
  //ROS_INFO("Start storing position msg0 ...");
  for(int i=0; i<lp; ++i)
    cout << msg0->data[i] << ' ';

  for(int j=0; j<lp; ++j)
    Pj0.push_back(msg0->data[j]) ;

  for(int j1=lp; j1<lj1; ++j1)
	 Pj1.push_back(msg0->data[j1]) ;

  for(int j2=lj1; j2<lj2; ++j2)
	 Pj2.push_back(msg0->data[j2]) ;

  for(int p=0; p<lp; ++p)
    cout <<"Pj0= %f" << Pj0[p] ;
  for(int p=0; p<lp; ++p)
    cout <<"Pj1= %f" << Pj1[p] ;
  for(int p=0; p<lp; ++p)
    cout <<"Pj2= %f" << Pj2[p] ;

  //Create an arm controller action client to move the GRASberry's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  waypoints_arm_goal(arm_goal);
  //Sends the command to start the given trajectory 1s from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);

}



// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "traj_control");

  ROS_INFO("Starting graspberry_traj_control application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Define a subscriber to subscribe to array of data from python node 
  ros::Subscriber sub0_traj = nh.subscribe("get_Joint0Traj", 1000, prompCallback0);
  ros::spin() ;

  // while (ros::ok()){
 	// ROS_INFO("Got to the end");
 	// //Create an arm controller action client to move the GRASberry's arm
  // 	arm_control_client_Ptr ArmClient;
  // 	createArmClient(ArmClient);
  // 	control_msgs::FollowJointTrajectoryGoal arm_goal ;
  // 	waypoints_arm_goal(arm_goal);
  // 	//Sends the command to start the given trajectory 1s from now
  // 	arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  // 	ArmClient->sendGoal(arm_goal);
 	// ros::spinOnce() ;}

  return 0 ;
}
