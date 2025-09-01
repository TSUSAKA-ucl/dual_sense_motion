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
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(test_args src/test_args.cpp)
target_include_directories(test_args PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(test_args PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  test_args
  "rclcpp"
)
install(TARGETS test_args
  DESTINATION lib/${PROJECT_NAME})


add_executable(dual_sense_motion src/dual_sense_motion.cpp)
ament_target_dependencies(dual_sense_motion std_srvs)
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

add_executable(imu_joy_subscriber src/imu_joy_subscriber.cpp)
target_include_directories(imu_joy_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIR}
  )
target_compile_features(imu_joy_subscriber PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  imu_joy_subscriber
  "rclcpp"
  "sensor_msgs"
  "tf2_ros"
)
install(TARGETS imu_joy_subscriber
  DESTINATION lib/${PROJECT_NAME})


set(LAUNCH_FILES
  launch/dual_sense_pose.launch
  launch/dual_sense_for_moveit.launch
  launch/dual_sense_default.launch
)
set(RVIZ_FILES rviz/dual_sense_motion.rviz)
install(FILES ${LAUNCH_FILES}
  DESTINATION share/${PROJECT_NAME}/launch
)
# rvizファイルはrvizが保存した時にためにsymlinkにしておく
set(RVIZ_INSTALL_DIR share/${PROJECT_NAME}/rviz)
install(DIRECTORY DESTINATION ${RVIZ_INSTALL_DIR}) # 空のdirectory生成
foreach(src ${RVIZ_FILES})
  file(RELATIVE_PATH rel_src	# 相対パスを計算する
    "${CMAKE_INSTALL_PREFIX}/${INSTALL_DIR}/${RVIZ_INSTALL_DIR}"   # 基準:
    "${CMAKE_SOURCE_DIR}/${src}"               # 元のファイル（絶対パス）
  )
  get_filename_component(fname ${src} NAME)
  install(CODE "
    execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
      \"${rel_src}\"
      \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${RVIZ_INSTALL_DIR}/${fname}\")
  ")
endforeach()


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
