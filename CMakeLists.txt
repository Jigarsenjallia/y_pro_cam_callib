cmake_minimum_required (VERSION 2.6)
project (y_pro_cam_callib)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

find_package(OpenCV REQUIRED )

set(BUILD_SHARED_LIBS ON)

add_executable(
  camera_calibration
  src/camera_calibration.cpp
)

target_link_libraries(
 camera_calibration
 ${OpenCV_LIBS}
)






