set(SOURCES
    "src/mavlink_usb_importer.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/mavlink_usb_importer.h"
)

include_directories(include)
add_library(mavlink_usb_importer MODULE ${SOURCES} ${HEADERS})
target_link_libraries(mavlink_usb_importer PRIVATE lmscore)
