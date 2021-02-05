#include "graspberry_robot/CartesianArm.h"
#include "graspberry_robot/SingleArm.h"
#include "graspberry_robot/ScaraArm.h"

using namespace can;

int main(int argc, char **argv) {
  ros::init(argc, argv, "graspberry_hardware_interface");
  ros::NodeHandle nh;

  std::string robot_name;

  GRASPberryArm* robot;

  //detect the robot name
  if (ros::param::get("~robot_name", robot_name)) {  // get param "robot_name" value from server and stores it in variable robot_name
  	if (robot_name == "cartesian_arm") {
  		robot = new CartesianArm();
  	} else if (robot_name == "single_arm") {
  		robot = new SingleArm();
  	} 
      else if (robot_name == "scara_arm") {
      robot = new ScaraArm() ; // class ScaraArm
    }
      else {
  		ROS_ERROR("robot_name '%s': unknown type", robot_name.c_str());
  		return -1;
  	}
  } else {
      ROS_ERROR("Failed to get param 'robot_name'");
      return -1;
  }

  double loop_hz = 50.0; //default rate

  ros::param::get(robot_name + "/hardware_interface/loop_hz", loop_hz);  // update loo_hz with the loop_hz param loaded on the server

  std::cerr << "Loop rate: " << loop_hz << " Hz" << std::endl;

  ros::Rate loop_rate(loop_hz);

  //initialise the communication with the arm
  try {
    robot->init();
  }
  catch(const std::runtime_error& e) {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }

  controller_manager::ControllerManager cm(robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  try {
    while (ros::ok()) {
      robot->read();  // read_status
      cm.update(robot->get_time(), robot->get_period());
      robot->write(); // write_command
      loop_rate.sleep();
    } 
  }
  catch(const std::runtime_error& e) {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }

  spinner.stop();

  return 0;  
}
