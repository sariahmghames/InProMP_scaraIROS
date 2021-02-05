#ifndef CANOPEN_H
#define CANOPEN_H

#include <vector>
#include <memory>
#include <stdexcept>
#include <chrono>

namespace CANOpen {

	typedef std::vector<unsigned char> DataPacket;

	class Device {
	protected:
		double time_out;

	public:
		Device(double _time_out) : 
		time_out(_time_out) {}

		virtual void Init() = 0;

		virtual void Close() = 0;

		virtual void Write(int can_id, const DataPacket& data) = 0;

		virtual void Read(int can_id, DataPacket& data) = 0;

		//read with timeout and check for message_id (bytes 1:3)
		virtual void ReadAck(int can_id, DataPacket& data) {
	        Write(can_id, data);

	        typedef std::chrono::high_resolution_clock Time;
	        typedef std::chrono::duration<double> Duration; 
	        auto start = Time::now();
	        DataPacket response = data;

	        while (Duration(Time::now() - start) < Duration(time_out)) {
	        	try {

		        	Read(can_id, response);
	        	}
		        catch (std::runtime_error&) {
		        	//try again 
		        	continue;
		        }

                if (std::equal(data.begin()+1, data.begin()+3, response.begin()+1)) {
                	data = response;
                	return;
                }
	        }

	        throw std::runtime_error("CANOPen::Device::ReadAck, no packets received, time out.");
		}

		//read with timeout and check for message_id (bytes 1:3)
		virtual void WriteAck(int can_id, const DataPacket& data) {
	        DataPacket request = data;

	        try {
	            ReadAck(can_id, request);
	        }
	        catch (std::runtime_error& err) {
	            throw std::runtime_error("CANOPen::Device::WriteAck, wrong response, time out.");
	        }
		}
	};

	typedef std::shared_ptr<Device> DevicePtr;

	enum NMT {
		StartRemoteNode = 0x01,
		StopRemoteNode = 0x02
	};

	enum IntType {
		INT8,
		INT16,
		INT32
	};

	class Interface {
	protected:
		DevicePtr device;
		int can_id;

	public:
		Interface(DevicePtr _device, int _can_id) :
		device(_device), can_id(_can_id) {}

		void Write(int can_id, const DataPacket& data) {
			device->Write(can_id, data);
		}

		void WriteAck(int can_id, const DataPacket& data) {
			device->WriteAck(can_id, data);
		}

		void Read(int can_id, DataPacket& data) {
			device->ReadAck(can_id, data);
		}

		void Write(const DataPacket& data) {
			Write(can_id, data);
		}

		void WriteAck(const DataPacket& data) {
			WriteAck(can_id, data);
		}

		void Read(DataPacket& data) {
			Read(can_id, data);
		}

		//write NMT command
		void Write(NMT value) {
        	DataPacket data(2);
        	data[0] = value;
        	data[1] = can_id;
        	Write(0x000, data);
		}

        //prepare uint32_t/int32_t value in a little-endian order
        void Write(const DataPacket& request, uint32_t value) {
        	DataPacket data = request;
            data[4] = value & 0xFF;
            data[5] = (value >> 8) & 0xFF;
            data[6] = (value >> 16) & 0xFF;
            data[7] = (value >> 24) & 0xFF;
            WriteAck(data);
        }

        uint32_t Read(const DataPacket& request) {
        	DataPacket data = request;
            Read(data);
            return (uint32_t)(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
        }

        int32_t Read(const DataPacket& request, IntType type) {
        	DataPacket data = request;
            Read(data);
            if (type == IntType::INT8)
            	return (int8_t)data[4];
            else if (type == IntType::INT16)
            	return (int16_t)(data[4] | (data[5] << 8));
            else
            	return (int32_t)(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
        }
	};

	namespace CiA402 {
		namespace Command {
		    const DataPacket ControlWord = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
		    const DataPacket StatusWord = {0x4B, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
		    const DataPacket ModeOfOperation = {0x2F, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
		    const DataPacket ModeOfOperationDisplay = {0x4F, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
		    const DataPacket PositionActualValue = {0x43, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
		    const DataPacket VelocityActualValue = {0x43, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
		    const DataPacket TargetPosition = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
		    const DataPacket GetTargetPosition = {0x43, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
		    const DataPacket ProfileVelocity = {0x23, 0x81, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};        
		    const DataPacket ProfileAcceleration = {0x2B, 0x83, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};        
		    const DataPacket ProfileDeceleration = {0x2B, 0x84, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}; 
		    const DataPacket ZeroReturnMode = {0x2F, 0x98, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}; 
		    const DataPacket ZeroReturnSpeed = {0x23, 0x99, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00}; 
		    const DataPacket ZeroReturnSwitchSpeed = {0x23, 0x99, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00}; 
		    const DataPacket ZeroReturnAccDec = {0x2B, 0x9A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}; 
		}

		class Interface : public CANOpen::Interface {
		public:
			Interface(DevicePtr device, int can_id) :
			CANOpen::Interface(device, can_id) {}

	        /// set target location in increments
	        void TargetPosition(int32_t value) {
	            Write(Command::TargetPosition, value);
	        }

	        int32_t TargetPosition() {
	            return Read(Command::GetTargetPosition, IntType::INT32);	        	
	        }

	        // set desired velocity in revolutions/s rps
	        void ProfileVelocity(uint32_t value) {
	            Write(Command::ProfileVelocity, value);
	        }

	        // set desired acceleration in revolutions/s rps
	        void ProfileAcceleration(uint16_t value) {
	            Write(Command::ProfileAcceleration, value);
	        }

	        // set desired deceleration in revolutions/s rps
	        void ProfileDeceleration(uint16_t value) {
	            Write(Command::ProfileDeceleration, value);
	        }

	        /// get the actual position in increments
	        int32_t Position() {
	            return Read(Command::PositionActualValue, IntType::INT32);
	        }

	        /// get the actual velocity in revolutions/s rps
	        int32_t Velocity() {
	            return Read(Command::VelocityActualValue, IntType::INT32);
	        }
		};
	}
}

#endif