#ifndef MAVLINK_USB_IMPORTER_H
#define MAVLINK_USB_IMPORTER_H

#include <lms/datamanager.h>
#include <lms/module.h>

class mavlink_usb_importer : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
};

#endif // MAVLINK_USB_IMPORTER_H
