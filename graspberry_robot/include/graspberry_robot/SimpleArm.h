#include "GRASPberryArm.h"
#include "JMCController.h"
#include "SocketCANDevice.h"
//#include "CANalystIIDevice.h" test with CANalystII device
#include <dynamic_reconfigure/server.h>
#include <graspberry_robot/GraspberryArmConfig.h>

class SimpleArm : public GRASPberryArm {
	enum State {
		UNINITIALISED,
		POSITION,
		HOMING
	};

	CANOpen::DevicePtr device;
    std::vector<CANOpen::JMCController*> joint_controller; //x, y, z axis controllers
    const std::vector<double> cob_id = {0x606};
    const std::vector<uint8_t> homing_mode = {0x6};
    const double incprev = 4000.0; //increments per revolution
    int position = 0;
    double time_out = 0.1;
    State state;
    typedef dynamic_reconfigure::Server<graspberry_robot::GraspberryArmConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server;    

    /// parameters
    std::vector<double> profile_vel, profile_acc, profile_dec, homing_vel;
    bool homing_at_init;

    //
    std::vector<uint16_t> current_status = {0, 0, 0};
public:

	CartesianArm() : 
	GRASPberryArm(), state(State::UNINITIALISED) {
		profile_vel = {0.0, 0.0, 0.0};
		profile_acc = {0.0, 0.0, 0.0};
		profile_dec = {0.0, 0.0, 0.0};
		homing_vel = {0.0, 0.0, 0.0};
		reconfigure_server = boost::make_shared<ReconfigureServer>();
		reconfigure_server->setCallback(boost::bind(&CartesianArm::reconfCallback, this, _1, _2));
	}

	~CartesianArm() {
		if (device)
			device->Close();
	}

	virtual void init() {

		device = std::make_shared<CANOpen::SocketCANDevice> (time_out);

		device->Init();

		for (int i = 0; i < cob_id.size(); i++)
			joint_controller.push_back(new CANOpen::JMCController(device, cob_id[i]));

        //start remote node
		for (int i = 0; i < joint_controller.size(); i++)
			joint_controller[i]->StartRemoteNode();

		//enable controller
		for (int i = 0; i < joint_controller.size(); i++) {
            joint_controller[i]->ControlWord(CANOpen::JMCController::ControlWordCommand::Shutdown); //shutdown
            joint_controller[i]->ControlWord(CANOpen::JMCController::ControlWordCommand::SwitchOn); //switched on
            joint_controller[i]->ControlWord(CANOpen::JMCController::ControlWordCommand::EnableOperation); //Operation Enabled
        }

        //set position control mode
        for (int i = 0; i < joint_controller.size(); i++) {
        	joint_controller[i]->ModeOfOperation(CANOpen::JMCController::ModeOfOperationValue::PositionMode);
        	joint_controller[i]->ProfileVelocity(profile_vel[i]);
        	joint_controller[i]->ProfileAcceleration(profile_acc[i]);
        	joint_controller[i]->ProfileDeceleration(profile_dec[i]);
        }

        read();
    }

    virtual void zero() {
    }


    virtual void read() {
    	ros::Time now = ros::Time::now();

	  	//update the state

    	for (int i = 0; i < joint_controller.size(); i++) {
    		pos[i] = joint_controller[i]->Position();
    		vel[i] = joint_controller[i]->Velocity();
    		eff[i] = 0.0;	  		
    	}

    	period = now - last_update;
    	last_update = now;
    }

    virtual void write() {
	  	//position controller
    	for (int i = 0; i < joint_controller.size(); i++) {
    		if (fabs(cmd[i] - pos[i]) > 1e-6) {
    			joint_controller[i]->SetPosition(cmd[i]);
    		}	  		
    	}
    }

    void reconfCallback(graspberry_robot::GraspberryArmConfig& config, uint32_t) {

    	std::cerr << "Reconfigure!" << std::endl;

  		//profile velocity
    	std::vector<double> profile_vel_cfg = {config.profile_vel_0, config.profile_vel_1, config.profile_vel_2};

    	for (int i = 0; i < profile_vel.size(); i++) {
    		if (profile_vel[i] != profile_vel_cfg[i]) {
    			profile_vel[i] = profile_vel_cfg[i];
    			if (state != State::UNINITIALISED)
    				joint_controller[i]->ProfileVelocity(profile_vel[i]);
    		}
    	}

  		//profile accceleration
    	std::vector<double> profile_acc_cfg = {config.profile_acc_0, config.profile_acc_1, config.profile_acc_2};

    	for (int i = 0; i < profile_acc.size(); i++) {
    		if (profile_acc[i] != profile_acc_cfg[i]) {
    			profile_acc[i] = profile_acc_cfg[i];
    			if (state != State::UNINITIALISED)
    				joint_controller[i]->ProfileVelocity(profile_acc[i]);
    		}
    	}

  		//profile decelearion
    	std::vector<double> profile_dec_cfg = {config.profile_dec_0, config.profile_dec_1, config.profile_dec_2};

    	for (int i = 0; i < profile_dec.size(); i++) {
    		if (profile_dec[i] != profile_dec_cfg[i]) {
    			profile_dec[i] = profile_dec_cfg[i];
    			if (state != State::UNINITIALISED)
    				joint_controller[i]->ProfileVelocity(profile_dec[i]);
    		}
    	}

  		//homing at init
    	homing_at_init = config.homing;

  		//profile decelearion
    	std::vector<double> homing_vel_cfg = {config.homing_vel_0, config.homing_vel_1, config.homing_vel_2};

    	for (int i = 0; i < homing_vel.size(); i++) {
    		if (homing_vel[i] != homing_vel_cfg[i]) {
    			homing_vel[i] = homing_vel_cfg[i];
    		}
    	}	
    }
};
