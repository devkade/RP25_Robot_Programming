cmake_minimum_required(VERSION 3.5)
project(service_exam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(rosidl_default_generators REQUIRED)

# 1. srv 파일 → C++ 코드 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyService.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(server src/service_exam_server.cpp)
ament_target_dependencies(server rclcpp)
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(client src/service_exam_client.cpp)
ament_target_dependencies(client rclcpp)
rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()


