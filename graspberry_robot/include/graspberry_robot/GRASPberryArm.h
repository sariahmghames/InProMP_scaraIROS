#ifndef GRASPBERRY_ARM_H
#define GRASPBERRY_ARM_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <math.h>

class GRASPberryArm : public hardware_interface::RobotHW, public hardware_interface::HardwareInterface
{
public:
  GRASPberryArm() { 

    //detect joint names in the paramserver
    std::string robot_name;
    std::vector<std::string> joint_names;

    if (ros::param::get("~robot_name", robot_name)) {
      std::string joint_names_param = robot_name + "/hardware_interface/joints";
      ros::param::get(joint_names_param, joint_names);

      std::stringstream joint_s;
      joint_s << "Joint names in " << joint_names_param << ": ";
      for(int i =0; i < joint_names.size(); i++) {
        joint_s << joint_names[i];
        if (i != (joint_names.size()-1))
          joint_s << ", ";
      }
      ROS_INFO("%s", joint_s.str().c_str());
    }

    //define state and position interfaces
    cmd.resize(joint_names.size());
    pos.resize(joint_names.size());
    vel.resize(joint_names.size());
    eff.resize(joint_names.size());

    for(int i =0; i < joint_names.size(); i++) {
      hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
      jnt_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
      jnt_pos_interface.registerHandle(pos_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    registerInterface(this);

    for(int i =0; i < joint_names.size(); i++) {
      cmd[i] = 0.0;
      pos[i] = 0.0;
    }
    
    last_update = ros::Time::now();
    period = ros::Time::now() - last_update;
  }

  virtual void init() {
    // establish communication with the motor controllers, etc.
  }

  virtual void zero() {
    // we can add more funcitonality like zero-ing etc here and expose via service calls
  }

  virtual void read() {
    ros::Time now = ros::Time::now();

    // call the CAN stuff here to populate the private members from the encoders
    for (int i=0; i < pos.size(); i++) {
      vel[i] = 0.0;
      eff[i] = 0.0;
      pos[i] = cmd[i];
    }

    period = now - last_update;
    last_update = now;
  }

  virtual void write() {
    for (int i=0; i<pos.size(); i++) {
      if (fabs(cmd[i] - pos[i]) > 1e-6) {
        // call the CAN stuff here to set the position in the controller or execute other commands
        // e.g. VCI_Transmit(VCI_USBCAN2, 0, 0, send_buffer, 1);
        ROS_INFO("update pos");
      }
    }
  }

  ros::Time get_time() {
    return last_update;
  }

  ros::Duration get_period() {
    return period;
  }

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  std::vector<double> cmd, pos, vel, eff;
  ros::Time last_update;
  ros::Duration period;
};

#endif