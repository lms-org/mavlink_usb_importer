#ifndef MAVLINK_USB_IMPORTER_H
#define MAVLINK_USB_IMPORTER_H

#include <lms/module.h>

#include <thread>
#include <vector>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>

class mavlink_usb_importer : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
    
protected:
    bool initUSB();
    bool deinitUSB();
    bool setUSBConfig(int fd);
    bool isValidFD(int fd);
    
    /**
     * Set file descriptor to blocking I/O mode
     * @param fd The usb device descriptor
     * @param blocking Whether to block (true) or not (false)
     */
    bool setBlocking( int fd, bool blocking );
    
    /**
     * Receiver task executed in background as ReceiverThread
     */
    void receiver();
    
protected:
    //! Paths to USB device
    std::vector<std::string> paths;
    
    //! USB device handle
    int usb_fd;

    //! Out data channel (messages to be sent)
    lms::WriteDataChannel<Mavlink::Data> outChannel;
    
    //! In data channel (messages received)
    lms::WriteDataChannel<Mavlink::Data> inChannel;
    
    //! Receiver thread reading sensor data in the background
    std::thread receiverThread;
    
    //! Flag whether to stop receiving thread
    bool shouldStopReceiver;
    
    //! Mavlink receive status
    mavlink_status_t receiverStatus;
    
    //! Temporary message buffer
    Mavlink::Data messageBuffer;
    std::mutex messageBufferMutex;
    
};

#endif // MAVLINK_USB_IMPORTER_H
