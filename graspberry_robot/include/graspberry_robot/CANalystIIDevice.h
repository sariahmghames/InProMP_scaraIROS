#include "CANOpen.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include "controlcan.h" //usb2can library
#include <stdexcept>

namespace CANOpen {

    class CANalystIIDevice : public Device {

        static bool initialised;

    public:
        CANalystIIDevice(double time_out) : 
        Device(time_out) {
        }

        ~CANalystIIDevice() {
        }

        virtual void Init() {

            if (initialised)
                return;

            if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == STATUS_ERR) //open device
                throw std::runtime_error("CANalystIIDevice::Init, unable to open the device.");

            // Initialisation
            VCI_INIT_CONFIG config;
            config.AccCode = 0;
            config.AccMask = 0xFFFFFFFF;
            config.Filter = 1; //receive all data
            config.Timing0 = 0x00; //Baud rate 500 Kbps  0x03  0x1C
            config.Timing1 = 0x1C;
            config.Mode = 0; //standard mode

            if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) == STATUS_ERR)
                throw std::runtime_error("CANalystIIDevice::Init, VCI_InitCAN error.");

            if (VCI_StartCAN(VCI_USBCAN2, 0, 0) == STATUS_ERR)
                throw std::runtime_error("CANalystIIDevice::Init, VCI_StartCAN error.");

            initialised = true;
        }

        virtual void Close() {

            if (!initialised)
                return;

            if (VCI_ResetCAN(VCI_USBCAN2, 0, 0) == STATUS_ERR)
                throw std::runtime_error("CANalystIIDevice::Close, VCI_ResetCAN error.");

        //wait 100ms?
            if (VCI_CloseDevice(VCI_USBCAN2, 0) == STATUS_ERR)
                throw std::runtime_error("CANalystIIDevice::Close, VCI_CloseDevice error.");

        //soft USB plug out to resolve issues with the incorectly open usb device
        //check if VCI_ResetCAN resolves the issue.

            const char *filename = "/dev/usb2can";

            int fd = open(filename, O_WRONLY);
            if (fd < 0) {
                throw std::runtime_error("CANalystIIDevice::Close, Error opening output file");
            }

            int rc = ioctl(fd, USBDEVFS_RESET, 0);
            if (rc < 0) {
                throw std::runtime_error("CANalystIIDevice::Close, Error in ioctl");
            }

            close(fd);

            initialised = false;
        }

        ///Direct Write
        virtual void Write(int can_id, const DataPacket& data) {
            VCI_CAN_OBJ can_message;
            can_message.ID = can_id;
            can_message.DataLen = data.size();
            std::copy(data.begin(), data.end(), std::begin(can_message.Data));

            if (VCI_Transmit(VCI_USBCAN2, 0, 0, &can_message, 1) == STATUS_ERR)
                throw std::runtime_error("CANalystIIDevice::Write, VCI_Transmit error.");
        }

        //read with timeout and check for message_id (bytes 1:3)
        virtual void ReadAck(int can_id, DataPacket& data) {
            Write(can_id, data);

            typedef std::chrono::high_resolution_clock Time;
            typedef std::chrono::duration<double> Duration; 
            auto start = Time::now();
            DataPacket response = data;

            while (Duration(Time::now() - start) < Duration(time_out)) {


                int data_packet_nr = VCI_GetReceiveNum(VCI_USBCAN2, 0, 0);
                if (!data_packet_nr)
                    continue;

                VCI_CAN_OBJ packets[data_packet_nr];

                int data_packet_received = VCI_Receive(VCI_USBCAN2, 0, 0, packets, data_packet_nr, 0);

                //iterate through all received packets
                for (int i = 0; i < data_packet_received; i++) {
                    if (packets[i].DataLen == data.size()) {
                        if (std::equal(data.begin()+1, data.begin()+3, std::begin(packets[i].Data)+1)) {
                            std::copy(std::begin(packets[i].Data), std::end(packets[i].Data), response.begin());
                            return;
                        }
                    }
                }
            }

            throw std::runtime_error("CANalystIIDevice::ReadAck, no packets received, time out.");
        }
    };

    bool CANalystIIDevice::initialised = false;
}

