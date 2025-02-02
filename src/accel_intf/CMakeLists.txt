cmake_minimum_required(VERSION 3.8)
project(accel_intf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include/)


set(ACCELERATION_INTERFACE_COMMON_DEFINES -DMODEL=acceleration_interface
    -DNUMST=1
    -DNCSTATES=0
    -DHAVESTDIO
    -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
    -DCLASSIC_INTERFACE=0
    -DALLOCATIONFCN=0
    -DTID01EQ=0
    -DTERMFCN=0
    -DONESTEPFCN=1
    -DMAT_FILE=0
    -DMULTI_INSTANCE_CODE=1
    -DINTEGER_CODE=0
    -DMT=0)
set(ACCELERATION_INTERFACE_COMMON_COMPILER_FLAGS -fwrapv)

add_library(acceleration_interface_objects OBJECT src/acceleration_interface.cpp
    src/acceleration_interface_data.cpp)

set_target_properties(acceleration_interface_objects PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(acceleration_interface_objects PRIVATE ${ACCELERATION_INTERFACE_COMMON_DEFINES})
target_compile_options(acceleration_interface_objects PRIVATE ${ACCELERATION_INTERFACE_COMMON_COMPILER_FLAGS})

ament_auto_add_executable(accel_intf src/main.cpp src/acceleration_interface_node.cpp)

target_compile_definitions(accel_intf PRIVATE ${ACCELERATION_INTERFACE_COMMON_DEFINES})

target_compile_options(accel_intf PRIVATE ${ACCELERATION_INTERFACE_COMMON_COMPILER_FLAGS})
target_link_libraries(accel_intf acceleration_interface_objects)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_simulink_model test/test_graceful_and_parking_brake.cpp 
                                      test/test_throttle.cpp test/test_gear_change.cpp
                                      test/test_braking.cpp)
  target_link_libraries(test_simulink_model acceleration_interface_objects)

  ament_add_gtest(test_acceleration_interface test/test_acceleration_interface_node.cpp src/acceleration_interface_node.cpp)
  ament_target_dependencies(test_acceleration_interface rclcpp std_msgs raptor_dbw_msgs blackandgold_msgs deep_orange_msgs)
  target_link_libraries(test_acceleration_interface acceleration_interface_objects)
endif()

install(TARGETS accel_intf
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
