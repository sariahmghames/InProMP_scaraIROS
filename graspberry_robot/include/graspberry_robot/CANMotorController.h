#include <vector>
#include <iostream>
#include <iterator>
#include <chrono>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include "controlcan.h" //usb2can library

using namespace std;

namespace CANOpen {

    typedef vector<unsigned char> DataPacket;

    void Init() {

        if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == STATUS_ERR) //open device
            throw runtime_error("CANOpen::Init, unable to open the device.");

        // Initialisation
        VCI_INIT_CONFIG config;
        config.AccCode = 0;
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1; //receive all data
        config.Timing0 = 0x00; //Baud rate 500 Kbps  0x03  0x1C
        config.Timing1 = 0x1C;
        config.Mode = 0; //standard mode

        if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) == STATUS_ERR)
            throw runtime_error("CANOpen::Init, VCI_InitCAN error.");

        if (VCI_StartCAN(VCI_USBCAN2, 0, 0) == STATUS_ERR)
            throw runtime_error("CANOpen::Init, VCI_StartCAN error.");
    }

    void Shutdown() {

        const char *filename = "/dev/usb2can";
        int fd;
        int rc;

        fd = open(filename, O_WRONLY);
        if (fd < 0) {
            throw runtime_error("Error opening output file");
        }

        rc = ioctl(fd, USBDEVFS_RESET, 0);
        if (rc < 0) {
            throw runtime_error("Error in ioctl");
        }

        close(fd);

        if (VCI_CloseDevice(VCI_USBCAN2, 0) == STATUS_ERR)
            throw runtime_error("CANOpen::Shutdown, VCI_CloseDevice error.");
    }

    ostream& operator<< (ostream& out, const VCI_CAN_OBJ& packet) {
        out << "CAN ID: " << "0x" << setfill('0') << setw(4) << hex << packet.ID << "; ";
        for (int i = 0; i < packet.DataLen; i++)
            out << "0x" << setfill('0') << setw(2) << hex << (int)packet.Data[i] << " ";
        return out;
    }

    ///Simple write
    void Write(int can_id, const DataPacket& data) {
        VCI_CAN_OBJ can_message;
        can_message.ID = can_id;
        can_message.DataLen = data.size();
        copy(data.begin(), data.end(), begin(can_message.Data));

        if (VCI_Transmit(VCI_USBCAN2, 0, 0, &can_message, 1) == STATUS_ERR)
            throw runtime_error("CANOpen::Device::Write, VCI_Transmit error.");
    }

    /// Blocking read with the request specified as input
    /// and response overriding the original request
    /// this one has a time_out function implemented correctly
    void Read(int can_id, DataPacket& data, double time_out) {
        VCI_ClearBuffer(VCI_USBCAN2, 0, 0);
        Write(can_id, data);

        typedef std::chrono::high_resolution_clock Time;
        typedef std::chrono::duration<double> Duration; 
        auto start = Time::now();

        while (Duration(Time::now() - start) < Duration(time_out)) { 
            int data_packet_nr = VCI_GetReceiveNum(VCI_USBCAN2, 0, 0);
            if (!data_packet_nr)
                continue;

            VCI_CAN_OBJ packets[data_packet_nr];

            int data_packet_received = VCI_Receive(VCI_USBCAN2, 0, 0, packets, data_packet_nr, 100);

            //iterate through all received packets
            for (int i = 0; i < data_packet_received; i++) {
                cerr << packets[i] << endl;
                    //the following code checks if the response is the same up to 4 bytes
                    //this is ok for SDO messages (8 bytes) but it is not very general

                if (packets[i].DataLen == 8) {
                    copy(begin(packets[i].Data), end(packets[i].Data), data.begin());
                    usleep(5000);
                    return;
                }
            }
        }

        throw runtime_error("CANOpen::Device::Read, no packets received, time out.");       
    }

namespace CiA402 {
    const DataPacket ControlWord = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    const DataPacket StatusWord = {0x4B, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    const DataPacket ModeOfOperation = {0x2F, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    const DataPacket ModeOfOperationDisplay = {0x2F, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    const DataPacket PositionActualValue = {0x43, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    const DataPacket VelocityActualValue = {0x43, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    const DataPacket TargetPosition = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    const DataPacket ProfileVelocity = {0x23, 0x81, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};        
    const DataPacket ProfileAcceleration = {0x2B, 0x83, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};        
    const DataPacket ProfileDeceleration = {0x2B, 0x84, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};        

    enum ModeOfOperationValue {
        // The device supports three modes: Velocity mode, Position mode, and Homing/Zero Return mode.
        Position = 0x01,
        PulseDirection = 0x02,
        Velocity = 0x03,
        Torque = 0x04,
        Homing = 0x06
    };

    enum ControlWordCommand {
        Shutdown = 0x0006,
        SwitchOn = 0x0007,
        EnableOperation = 0x000F
    };

    enum StatusWordState {
        TargetReached = 0x0200
    };
}

class JMCController {
    int can_id;
    double time_out;
    const int drive_scaling_factor = 10; // The setting value is 10 times the actual value. For example, if the value is 100, the actual speed value is 10r/s

    public:
        JMCController(int _can_id, double _time_out = 0.1) :
        can_id(_can_id), time_out(_time_out) {}

        void ControlWord(CiA402::ControlWordCommand value) {
            DataPacket data = CiA402::ControlWord;
            WriteUINT16(data, value);            
        }

        // get current state of the drive
        bool StatusWord(CiA402::StatusWordState value) {
            DataPacket data = CiA402::StatusWord;
            uint16_t current_state = ReadUINT16(data);
            if (value == value & current_state)
                return true;
            else
                return false;
        }

        /// set operational mode to position control
        void ModeOfOperation(CiA402::ModeOfOperationValue value) {
            DataPacket data = CiA402::ModeOfOperation;
            data[4] = (unsigned char)value;
            Write(data);
        }

        /// get value of the operational mode
        CiA402::ModeOfOperationValue ModeOfOperation() {
            DataPacket data = CiA402::ModeOfOperationDisplay;
            Read(data);
            return (CiA402::ModeOfOperationValue)data[4];
        }

        /// set target location in increments
        void Position(int32_t value) {
            DataPacket data = CiA402::TargetPosition;
            WriteINT32(data, value);
        }

        /// get the actual position in increments
        int32_t Position() {
            DataPacket data = CiA402::PositionActualValue;
            return ReadINT32(data);
        }

        // set desired velocity in revolutions/s rps
        void ProfileVelocity(uint32_t value) {
            DataPacket data = CiA402::ProfileVelocity;
            value *= drive_scaling_factor;
            WriteUINT32(data, value);
        }

        /// get the actual velocity in revolutions/s rps
        int32_t Velocity() {
            DataPacket data = CiA402::VelocityActualValue;
            return (ReadINT32(data) / drive_scaling_factor);
        }

        // set desired acceleration in revolutions/s rps
        void ProfileAcceleration(uint16_t value) {
            DataPacket data = CiA402::ProfileAcceleration;
            value *= drive_scaling_factor;
            WriteUINT16(data, value);
        }

        // set desired deceleration in revolutions/s rps
        void ProfileDeceleration(uint16_t value) {
            DataPacket data = CiA402::ProfileDeceleration;
            value *= drive_scaling_factor;
            WriteUINT16(data, value);
        }

        virtual void Write(const DataPacket& data) {
            CANOpen::Write(can_id, data);
        }

        //prepare uint32 value in a little-endian order
        void WriteUINT32(DataPacket& data, uint32_t value) {
            data[4] = value & 0xFF;
            data[5] = (value >> 8) & 0xFF;
            data[6] = (value >> 16) & 0xFF;
            data[7] = (value >> 24) & 0xFF;
            Write(data);
        }

        //prepare uint16 value in a little-endian order
        void WriteUINT16(DataPacket& data, uint16_t value) {
            data[4] = value & 0xFF;
            data[5] = (value >> 8) & 0xFF;
            Write(data);
        }

        //prepare int32 value in a little-endian order
        void WriteINT32(DataPacket& data, int32_t value) {
            data[4] = value & 0xFF;
            data[5] = (value >> 8) & 0xFF;
            data[6] = (value >> 16) & 0xFF;
            data[7] = (value >> 24) & 0xFF;
            Write(data);
        }

        virtual void Read(DataPacket& data) {
            CANOpen::Read(can_id, data, time_out);
        }

        uint16_t ReadUINT16(DataPacket& data) {
            Read(data);
            uint16_t value = data[4] | (data[5] << 8);
            return value;
        }

        int32_t ReadINT32(DataPacket& data) {
            Read(data);
            int32_t value = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
            return value;            
        }
    };
}
