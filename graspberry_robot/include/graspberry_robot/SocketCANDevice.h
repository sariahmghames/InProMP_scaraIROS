#ifndef SOCKETCAN_DEVICE_H
#define SOCKETCAN_DEVICE_H

#include "CANOpen.h"
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/reader.h>
#include <stdexcept>

namespace CANOpen {

    class SocketCANDevice : public Device {

        can::SocketCANInterface* driver;
        can::ThreadedSocketCANInterfaceSharedPtr devicePtr;
        can::BufferedReader* reader;
        static bool initialised;

    public:
        SocketCANDevice(double time_out) : 
        Device(time_out), reader(0), driver(0) {
        }

        ~SocketCANDevice() {
        }

        virtual void Init() {

            if (initialised)
                return;

            devicePtr.reset(new can::ThreadedSocketCANInterface());

            if (!devicePtr->init("can0", false))
                throw std::runtime_error("SocketCANDevice::Init, unable to open the device.");

            reader = new can::BufferedReader();

            reader->listen((can::CommInterfaceSharedPtr)devicePtr);

            initialised = true;
        }

        virtual void Close() {

            if (!initialised)
                return;

            if (reader)
                delete(reader);

            devicePtr->shutdown();

            initialised = false;
        }

        ///Direct Write
        virtual void Write(int can_id, const DataPacket& data) {
            can::Header header(can_id, false, false, false);
            can::Frame message(header, data.size());

            std::copy(data.begin(), data.end(), std::begin(message.data));

            if (!devicePtr->send(message))
                throw std::runtime_error("SocketCANDevice::Write, unable to send data.");
        }

        ///Direct Read
        virtual void Read(int can_id, DataPacket& data) {
            can::Frame frame;

            if (reader->read(&frame, boost::chrono::milliseconds::zero())) {
                std::copy(std::begin(frame.data), std::end(frame.data), data.begin());
                return;
            }

            throw std::runtime_error("SocketCANDevice::Read, no packets received, time out.");       
        }
    };

    bool SocketCANDevice::initialised = false;
}

#endif