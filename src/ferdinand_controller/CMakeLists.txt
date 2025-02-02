cmake_minimum_required(VERSION 3.8)
project(ferdinand_controller)

#set(CMAKE_BUILD_TYPE RelWithDebInfo) # FIXME: Comment this

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(blackandgold_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Find Eigen package
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)


add_executable(ferdinand_controller src/ferdinand_controller.cpp)
ament_target_dependencies(ferdinand_controller blackandgold_msgs rclcpp std_msgs visualization_msgs raptor_dbw_msgs nav_msgs geometry_msgs deep_orange_msgs Eigen3)
target_include_directories(ferdinand_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ferdinand_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS ferdinand_controller
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
