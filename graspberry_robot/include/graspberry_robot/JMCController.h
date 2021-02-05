#ifndef JMC_CONTROLLER_H
#define JMC_CONTROLLER_H

#include "CANOpen.h"

namespace CANOpen {

	class JMCController : public CiA402::Interface {
	public:
		JMCController(DevicePtr device, int can_id) :
		CiA402::Interface(device, can_id) {}

	    enum ModeOfOperationValue {
	        // The device supports three modes: Velocity mode, Position mode, and Homing/Zero Return mode.
	        PositionMode = 0x01,
	        PulseDirectionMode = 0x02,
	        VelocityMode = 0x03,
	        TorqueMode = 0x04,
	        HomingMode = 0x06
	    };

	    enum ControlWordCommand {
	        Shutdown = 0x0006,
	        SwitchOn = 0x0007,
	        EnableOperation = 0x000F,
	        NewLocation = 0x001F,
	        NewLocationDirect = 0x023F,
	        EnableOperationDirect = 0x022F
	    };

	    enum StatusWordState {
	        MotorRunning = 1 << 9,
	        TargetReached = 1 << 10,
	        InPosition = 1 << 12
	    };

	    void StartRemoteNode() {
	    	Write(CANOpen::NMT::StartRemoteNode);
	    }

        void ControlWord(ControlWordCommand value) {
            Write(CiA402::Command::ControlWord, value);            
        }

        /// set operational mode to position control
        void ModeOfOperation(ModeOfOperationValue value) {
            Write(CiA402::Command::ModeOfOperation, value);
        }

        // get the current state of the drive
        uint16_t StatusWord() {
            return Read(CiA402::Command::StatusWord);
        }

        bool HomingCompleted() {
        	return StatusWord() & StatusWordState::InPosition;
        }

        /// get value of the operational mode
        ModeOfOperationValue ModeOfOperation() {
            return (ModeOfOperationValue)Read(CiA402::Command::ModeOfOperationDisplay, IntType::INT8);
        }

        /// set position in the position mode
        void SetPosition(int32_t value) {
        	//check if the current target position is different from the requested value
        	if (TargetPosition() != value) {        		
	        	uint16_t status = StatusWord();
	        	if (status & StatusWordState::InPosition) { //can set new position
		        	TargetPosition(value); //set new position

		        	if (status & StatusWordState::MotorRunning) { //if the motor is running set the next point 
		    	    	ControlWord(ControlWordCommand::NewLocationDirect);
		        	} else { // else just set a new point
		    	    	ControlWord(ControlWordCommand::NewLocation);
		        	}
		        	ControlWord(ControlWordCommand::EnableOperation); //clear the flag to execute the operation
	        	} else {
		        	std::cerr << "Cannot set new position" << std::endl;
	        	}
        	}
        }
	};
}

#endif