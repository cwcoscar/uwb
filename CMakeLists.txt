cmake_minimum_required(VERSION 3.0.2)
project(uwb)

add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  sensor_msgs
  uwb_ins_eskf_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(DIRECTORY msg)
# add_message_files(
#   FILES
#   uwb_raw.msg
#   uwb_ndt_compare.msg
# )
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
 CATKIN_DEPENDS 
 roscpp 
 rospy 
 std_msgs 
 message_runtime
 sensor_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(uwb_positioning src/main_positioning.cpp src/uwb_positioning.cpp src/uwb_class.cpp)

target_link_libraries(uwb_positioning ${catkin_LIBRARIES})

add_dependencies(uwb_positioning
  ${catkin_EXPORTED_TARGETS}
)

add_executable(uwb_ndt_comparison src/uwb_ndt_compare.cpp)

target_link_libraries(uwb_ndt_comparison ${catkin_LIBRARIES})

add_dependencies(uwb_ndt_comparison
  ${catkin_EXPORTED_TARGETS}
)

add_executable(uwb_calibration src/uwb_calibration.cpp)

target_link_libraries(uwb_calibration ${catkin_LIBRARIES})

add_dependencies(uwb_calibration
  ${catkin_EXPORTED_TARGETS}
)

add_executable(uwb_start_YCHIOT src/uwb_start.cpp)

option(SERIAL_BUILD_SHARED_LIBS "Build CppLinuxSerial shared library" OFF)

if (SERIAL_BUILD_SHARED_LIBS)
  set(LibType SHARED)
else()
  set(LibType STATIC)
endif()

file(GLOB_RECURSE CppLinuxSerial_SRC
        "src/SerialPort.cpp")

file(GLOB_RECURSE CppLinuxSerial_HEADERS
        "include/CppLinuxSerial/*.hpp")

add_library(CppLinuxSerial ${LibType} ${CppLinuxSerial_SRC} ${CppLinuxSerial_HEADERS})

target_link_libraries(uwb_start_YCHIOT CppLinuxSerial pthread ${catkin_LIBRARIES})

add_dependencies(uwb_start_YCHIOT
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/uwb_csv_recorder.py
  scripts/cal_std_mean.py
  scripts/cal_std_mean_tag_at_same_point.py
  scripts/preprocess.py
  scripts/ndt_csv_recorder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  uwb_start_YCHIOT
  uwb_calibration
  uwb_ndt_comparison
  uwb_positioning
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

