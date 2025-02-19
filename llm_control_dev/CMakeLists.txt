cmake_minimum_required(VERSION 3.8)
project(llm_control_dev)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Adicionar executável do nó C++
add_executable(llm_service_node src/llm_service_node.cpp)
ament_target_dependencies(llm_service_node rclcpp std_msgs)

install(TARGETS llm_service_node DESTINATION lib/${PROJECT_NAME})

# Instalar o nó Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/llm_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
