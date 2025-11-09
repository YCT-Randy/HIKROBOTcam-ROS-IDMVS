cmake_minimum_required(VERSION 3.0.2)
project(hik_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
  fiducial_msgs
)

## Find Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport cv_bridge
)

# 設定 Hikvision SDK library 的目錄
set(HIKVISION_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 找到所有共享函式庫
find_library(FORMAT_CONVERSION_LIB NAMES FormatConversion PATHS ${HIKVISION_SDK_LIB_DIR})
find_library(MEDIA_PROCESS_LIB NAMES MediaProcess PATHS ${HIKVISION_SDK_LIB_DIR})
find_library(MVCAMERA_LIB NAMES MvCameraControl PATHS ${HIKVISION_SDK_LIB_DIR})
find_library(MVCODEREADER_LIB NAMES MvCodeReaderCtrl PATHS ${HIKVISION_SDK_LIB_DIR})
find_library(MVGIGE_LIB NAMES MVGigEVisionSDK PATHS ${HIKVISION_SDK_LIB_DIR})
find_library(MVRENDER_LIB NAMES MVRender PATHS ${HIKVISION_SDK_LIB_DIR})
find_library(MVUSB_LIB NAMES MvUsb3vTL PATHS ${HIKVISION_SDK_LIB_DIR})

# 確保所有庫都被找到
if(NOT FORMAT_CONVERSION_LIB OR NOT MEDIA_PROCESS_LIB OR NOT MVCAMERA_LIB OR NOT MVCODEREADER_LIB OR NOT MVGIGE_LIB OR NOT MVRENDER_LIB OR NOT MVUSB_LIB)
    message(FATAL_ERROR "One or more Hikvision SDK libraries were not found in ${HIKVISION_SDK_LIB_DIR}. Please check your installation.")
endif()

find_package(OpenCV REQUIRED)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
file(GLOB LIB_FILES "${CMAKE_CURRENT_SOURCE_DIR}/lib/*")
file(COPY ${LIB_FILES} DESTINATION ${CATKIN_DEVEL_PREFIX}/lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
)

add_executable(hikcam_node 
  src/hikcam_node.cpp 
  src/hikcam_ros.cpp 
  src/hikcam.cpp
  src/hikcam_imgbuffer.cpp
  src/hikcam_infodecoder.cpp
  src/hikcam_tag.cpp
  src/hikcam_udp_mode.cpp
  src/hikcam_sdk_mode.cpp
)


target_link_libraries(hikcam_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${FORMAT_CONVERSION_LIB}
  ${MEDIA_PROCESS_LIB}
  ${MVCAMERA_LIB}
  ${MVCODEREADER_LIB}
  ${MVGIGE_LIB}
  ${MVRENDER_LIB}
  ${MVUSB_LIB}
)

# install(DIRECTORY lib/
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/lib
#     FILES_MATCHING PATTERN "*.so"
# )
install(TARGETS 
  hikcam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/hik_camera
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/hik_camera
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/hik_camera
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(FILES
        include/MvCodeReaderCtrl.h
        include/MvCodeReaderErrorDefine.h
        include/MvCodeReaderParams.h
        include/MvCodeReaderPixelType.h
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
  DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # FILES_MATCHING PATTERN "*.so"
)
