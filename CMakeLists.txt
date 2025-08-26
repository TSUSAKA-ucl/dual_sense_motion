cmake_minimum_required(VERSION 3.8)
project(dual_sense_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dual_sense_motion src/dual_sense_motion.cpp)
target_include_directories(dual_sense_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIR}
  )
target_compile_features(dual_sense_motion PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dual_sense_motion
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS dual_sense_motion
  DESTINATION lib/${PROJECT_NAME})

add_executable(simple_sub_imu src/simple_sub_imu.cpp)
target_include_directories(simple_sub_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIR}
  )
target_compile_features(simple_sub_imu PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  simple_sub_imu
  "rclcpp"
  "sensor_msgs"
)
install(TARGETS simple_sub_imu
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
