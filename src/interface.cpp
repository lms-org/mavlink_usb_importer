#include "mavlink_usb_importer.h"

extern "C" {
void* getInstance () {
    return new mavlink_usb_importer();
}
}
