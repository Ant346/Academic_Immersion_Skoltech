cmake_minimum_required(VERSION 3.8)
project(gige)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} include)
link_directories(${OpenCV_LBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable(
  ${PROJECT_NAME}_camera
  src/main.cxx
  src/Camera.cxx
  src/GigEManager.cxx
  include/Data.hxx
  include/Camera.hxx
  include/GigEManager.hxx
)

ament_target_dependencies(
  ${PROJECT_NAME}_camera
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

target_link_libraries(
  ${PROJECT_NAME}_camera
  ${DEPENDENCIES}
  rcl::rcl
  rclcpp::rclcpp
  ${OpenCV_LIBRARIES}
  MVSDK
  pthread
  tbb
)

install(TARGETS
  ${PROJECT_NAME}_camera
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