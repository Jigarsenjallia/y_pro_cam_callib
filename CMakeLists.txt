cmake_minimum_required (VERSION 2.6)
project (y_pro_cam_callib)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

find_package(OpenCV REQUIRED )

set(BUILD_SHARED_LIBS ON)

include_directories (
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(qt_hdrs
  include/y_pro_cam_callib.h
)

set(qt_srcs	  
  src/y_pro_cam_callib.cpp
)
 
set(qt_uis
  src/main_window.ui
  src/display.ui
)

set(qt_qrc
  res/resource.qrc
)

qt4_automoc(${qt_srcs})
qt4_add_resources(qt_qrc_srcs ${qt_qrc})
qt4_wrap_cpp(qt_moc_srcs ${qt_hdrs})
qt4_wrap_ui(uis_h ${qt_uis})  

add_executable (${PROJECT_NAME} 
  ${qt_srcs}
  ${qt_moc_srcs}
  ${uis_h}
  ${qt_qrc_srcs}
)

target_link_libraries (${PROJECT_NAME} 
  ${QT_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(
  camera_calibration
  src/camera_calibration.cpp
)

target_link_libraries(
  camera_calibration
  ${OpenCV_LIBS}
)



