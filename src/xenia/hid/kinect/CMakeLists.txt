add_library(xenia-hid-kinect STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_audio.h
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_camera.h
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_camera_recording.cc
)

# The camera is read through libusb where third_party builds it: Windows
# (UsbDk or WinUSB) and Linux (usbfs). Other platforms report no camera.
if(WIN32 OR (UNIX AND NOT APPLE))
  target_sources(xenia-hid-kinect PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_audio_libusb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_camera_libusb.cc
  )
  target_link_libraries(xenia-hid-kinect PUBLIC libusb)

  # Streams from the camera on the host and reports the frame rate, to tell a
  # USB problem from one in the emulated kernel.
  add_executable(xenia-kinect-camera-test
    ${CMAKE_CURRENT_SOURCE_DIR}/kinect_camera_test_main.cc
  )
  if(WIN32)
    target_sources(xenia-kinect-camera-test PRIVATE
      ${PROJECT_SOURCE_DIR}/src/xenia/base/console_app_main_win.cc)
  else()
    target_sources(xenia-kinect-camera-test PRIVATE
      ${PROJECT_SOURCE_DIR}/src/xenia/base/console_app_main_posix.cc)
  endif()
  target_link_libraries(xenia-kinect-camera-test PRIVATE
    fmt xenia-base xenia-hid-kinect)
  xe_target_defaults(xenia-kinect-camera-test)
else()
  target_sources(xenia-hid-kinect PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_camera_posix.cc
  )
endif()
target_link_libraries(xenia-hid-kinect PUBLIC xenia-base)
xe_target_defaults(xenia-hid-kinect)
