cmake_minimum_required(VERSION 3.8)
project(group23_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# include folder whish has all the header files
include_directories(include/group23_final)

# Add this line to include the config folder
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# target_link_libraries(navigator_23 ${nav2_msgs_LIBRARIES})

#-----------------------------
# C++
#-----------------------------
add_executable(navigator_23 src/navigator_23.cpp src/utils.cpp)
ament_target_dependencies(navigator_23 ros2_aruco_interfaces mage_msgs sensor_msgs rclcpp rclcpp_action rclcpp_components nav2_msgs geometry_msgs ${FRAME_DEMO_INCLUDE_DEPENDS})

install(TARGETS navigator_23
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#-----------------------------
# Installation
#-----------------------------


install(TARGETS
navigator_23
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config
  DESTINATION share/${PROJECT_NAME}/
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
