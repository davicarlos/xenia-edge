add_library(xenia-hid-kinect STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_camera.h
)

# The camera is read through libusb where third_party builds it: Windows
# (UsbDk or WinUSB) and Linux (usbfs). Other platforms report no camera.
if(WIN32 OR (UNIX AND NOT APPLE))
  target_sources(xenia-hid-kinect PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_camera_libusb.cc
  )
  target_link_libraries(xenia-hid-kinect PUBLIC libusb)
else()
  target_sources(xenia-hid-kinect PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_camera_posix.cc
  )
endif()
target_link_libraries(xenia-hid-kinect PUBLIC xenia-base)
xe_target_defaults(xenia-hid-kinect)
