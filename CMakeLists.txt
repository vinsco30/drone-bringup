cmake_minimum_required(VERSION 3.8)
project(gz_drone_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable( odom_republisher_simu src/odom_republisher_simu.cpp )
ament_target_dependencies( odom_republisher_simu rclcpp nav_msgs tf2 tf2_ros Eigen3 px4_msgs rclpy )
install(TARGETS odom_republisher_simu DESTINATION lib/${PROJECT_NAME})

add_executable( odom_republisher_zed src/odom_republisher_zed.cpp )
ament_target_dependencies( odom_republisher_zed rclcpp nav_msgs tf2 tf2_ros Eigen3 px4_msgs rclpy )
install(TARGETS odom_republisher_zed DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
    )

# Install config files
install(DIRECTORY 
    config
    DESTINATION share/${PROJECT_NAME}
    ) 

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
