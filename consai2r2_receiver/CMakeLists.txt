cmake_minimum_required(VERSION 3.5)
project(consai2r2_receiver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(consai2r2_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Protobuf
include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

protobuf_generate_cpp(PROTO_CPP PROTO_H
  include/${PROJECT_NAME}/proto/messages_robocup_ssl_wrapper.proto
  include/${PROJECT_NAME}/proto/messages_robocup_ssl_geometry.proto
  include/${PROJECT_NAME}/proto/messages_robocup_ssl_detection.proto
  include/${PROJECT_NAME}/proto/messages_robocup_ssl_robot_status.proto
  include/${PROJECT_NAME}/proto/messages_robocup_ssl_refbox_log.proto
  include/${PROJECT_NAME}/proto/referee.proto
  include/${PROJECT_NAME}/proto/game_event.proto
)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
add_executable(referee_receiver
  src/referee_receiver.cpp
  ${PROTO_CPP}
  ${PROTO_H}
)
ament_target_dependencies(
  referee_receiver
  "rclcpp"
  "std_msgs"
  "consai2r2_msgs"
)
target_link_libraries(referee_receiver ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES})


install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS referee_receiver
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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