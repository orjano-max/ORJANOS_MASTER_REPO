cmake_minimum_required(VERSION 3.8)
project(scene_geometry_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(scene_geometry_publisher 
  src/scene_geometry_publisher.cpp)

target_include_directories(
  scene_geometry_publisher 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


target_compile_features(scene_geometry_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  scene_geometry_publisher
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS scene_geometry_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scenes DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY include DESTINATION share/${PROJECT_NAME})

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
