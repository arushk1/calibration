cmake_minimum_required(VERSION 2.8.3)
project(calibration)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  tf
  image_proc
  message_generation
  visp_bridge
)

find_package(VISP REQUIRED)


add_definitions(${VISP_DEFINITIONS})

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


# Generate messages and services.
add_message_files(
  DIRECTORY
    msg
  FILES
    TransformArray.msg
  )
add_service_files(
  DIRECTORY
    srv
  FILES
    compute_effector_camera.srv
    compute_effector_camera_quick.srv
    reset.srv
  )
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
)

add_library(calibration_common
  src/names.cpp
  include/names.h
)
target_link_libraries(calibration_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )

add_executable(calibration_calibrator
  src/calibrator.cpp
  include/calibrator.h
  src/calibrator_main.cpp
)
target_link_libraries(calibration_calibrator
  calibration_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )
add_dependencies(calibration_calibrator
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp)

add_executable(calibration_client
  src/client.cpp
  include/client.h
  src/client_main.cpp
)
target_link_libraries(calibration_client
  calibration_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )
add_dependencies(calibration_client
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp)

  add_executable(sync_publisher
    src/sync_publisher.cpp
    include/client.h
    src/client.cpp)

  target_link_libraries(sync_publisher
    calibration_common
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    )


install(
  TARGETS
    sync_publisher
    calibration_calibrator
    calibration_client
    calibration_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
