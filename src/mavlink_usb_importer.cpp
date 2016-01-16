#include "mavlink_usb_importer.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <termios.h>

#ifndef __APPLE__
  #include <linux/usbdevice_fs.h>
#endif

bool mavlink_usb_importer::initialize() {

    paths = config().getArray<std::string>("path");

    // Setup datachannels
    inChannel = writeChannel<Mavlink::Data>("MAVLINK_IN");
    outChannel = writeChannel<Mavlink::Data>("MAVLINK_OUT");

    if(!initUSB()){
        return false;
    }

    return true;
}

bool mavlink_usb_importer::deinitialize() {
    logger.info("deinitialize") << "Closing MAVlink USB Importer";
    
    // Set flag to stop receiver
    shouldStopReceiver = true;
    
    // Wait for receiver thread to be finished with current cycle
    receiverThread.join();
    
    // Deinit interface properly
    deinitUSB();

    return true;
}

bool mavlink_usb_importer::cycle() {
    // Send messages
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    //TODO count errors
    for(const auto& msg : *outChannel ){
        while(true){
            uint16_t numBytes = mavlink_msg_to_send_buffer(buffer, &msg);
            logger.debug("cycle")<<"trying to send "<<numBytes;
            ssize_t numBytesTransmitted = write(usb_fd, buffer, numBytes);
            logger.debug("cycle")<<"I sent "<<numBytesTransmitted;
            if( numBytes == numBytesTransmitted ){
                logger.debug("cycle") << "Sent message with " << numBytes << " bytes!";
                break;
            } else if( numBytesTransmitted < 0 ){
                logger.perror("cycle") << "Unable to send message";
                isValidFD(usb_fd);
            } else{
                logger.warn("cycle") << "Tried to send message with " << numBytes << "bytes, but transmitted " << numBytesTransmitted;
                isValidFD(usb_fd);
            }
            lms::Time::fromMillis(1).sleep();
        }
    }
    
    // Clear all messages from channel
    outChannel->clear();
    
    // Move message buffer to inChannel
    messageBufferMutex.lock();
    
    // TODO: use move semantics
    *inChannel = messageBuffer;
    //logger.debug("new messages: ")<<messageBuffer.messages.size();
    messageBuffer.clear();
    
    messageBufferMutex.unlock();
    
    return true;
}

void mavlink_usb_importer::receiver()
{
    mavlink_message_t msg;
    
    while( usb_fd >= 0 && !shouldStopReceiver )
    {
        // read byte
        char buf;
        ssize_t numBytes = read(usb_fd, &buf, 1);
        if(numBytes == 1)
        {
            if(mavlink_parse_char(0, buf, &msg, &receiverStatus))
            {
                // new message received, put message in temporary buffer
                messageBufferMutex.lock();
                messageBuffer.add(msg);
                messageBufferMutex.unlock();
                logger.debug("receive") << "Received Message!";
            }
        }
    }
    
    logger.warn("receiverThread") << "Terminating receiver thread";
}

bool mavlink_usb_importer::isValidFD(int fd) {
#ifdef __APPLE__
    // Try to read message
    char _buffer = 0;
    int result = read(usb_fd, &_buffer, 1);
    
    if(result == 1) {
        return true;
    }
#else
    /// Sende Anfrage damit ioctl errno neu setzt
    int rc = ioctl(fd, USBDEVFS_CONNECTINFO, 0);
    
    /// Wenn rc != 0 gab es einen Fehler
    if (rc == 0){
        return true;
    }
#endif
    
    logger.perror("is_valid_fd") << "Error in ioctl: Trying to reconnect";
    /// Haben wir einen Input/Output error? -> usb neu initialisieren
    if(errno == EIO || errno == EBADF || errno == ENXIO) {
        close(fd);
        while(!initUSB()) {
            usleep(100);
        }
    }

    return false;
}

bool mavlink_usb_importer::initUSB(){

    for(const auto& path : paths)
    {
        usb_fd = open(path.c_str(), O_RDWR | O_NDELAY);
        if (usb_fd < 0) {
            logger.perror("init") << "Open USB device at " << path;
            continue;
        } else {
            break;
        }
    }

    if( usb_fd <= 0 )
    {
        // No valid device found
        return false;
    }
    
    // Set configuration
    setUSBConfig(usb_fd);
    
    // Set blocking I/O
    setBlocking( usb_fd, true );
    
    
    // http://arduino.cc/en/Main/ArduinoBoardNano
    //
    // When the Nano is connected to either a computer running
    // Mac OS X or Linux, it resets each time a connection is made
    // to it from software (via USB). For the following half-second
    // or so, the bootloader is running on the Nano.
    //
    // [...] make sure that the software with which it communicates
    // waits a second after opening the connection and before sending
    // this data.
     
    sleep( config().get<unsigned int>("init_sleep", 0) );

    tcflush(usb_fd,TCIOFLUSH);
    
    logger.info("init") << "Initialized USB device connection";

    // Start receiver thread
    if(receiverThread.joinable()) {
        receiverThread.join();
    }
    shouldStopReceiver = false;
    receiverThread = std::thread(&mavlink_usb_importer::receiver, this);

    return true;
}

bool mavlink_usb_importer::setUSBConfig(int fd)
{
    struct termios tio;
    
    ///Termios
    //further reading http://en.wikibooks.org/wiki/Serial_Programming/termios
    tcgetattr(fd, &tio);

    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    tio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // tio.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    tio.c_oflag = 0;
    //
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    tio.c_cflag &= ~(CSIZE | PARENB);
    tio.c_cflag |= CS8;
    
    // Ignore modem control lines
    tio.c_cflag |= (CLOCAL | CREAD );
    
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0; //0

    // first set speed and then set attr!!!
    if(cfsetispeed(&tio, B115200) < 0 || cfsetospeed(&tio, B115200) < 0) {
        logger.perror("init") << "Baud rate";
    }

    if(tcsetattr(fd, TCSANOW, &tio) < 0) {
        logger.perror("init") << "SET ATTR";
    }
    
    return true;
}

bool mavlink_usb_importer::deinitUSB()
{
    if( usb_fd >= 0 )
    {
        close(usb_fd);
    }
    
    usb_fd = -1;
    
    return true;
}

bool mavlink_usb_importer::setBlocking( int fd, bool blocking )
{
    int flags = fcntl(fd, F_GETFL);
    if( blocking )
    {
        // Set to blocking
        flags = flags & ( ~O_NONBLOCK );
    } else {
        // Set to non-blocking
        flags = flags | O_NONBLOCK;
    }
    
    auto result = fcntl(fd, F_SETFL, flags);
    
    if(result == -1)
    {
        logger.perror("setBlocking");
        return false;
    }
    return true;
}
