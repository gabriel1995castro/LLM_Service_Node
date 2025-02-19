cmake_minimum_required(VERSION 3.5)
project(llm_control_dev)

# Adiciona suporte ao C++14 (ou versão superior)
set(CMAKE_CXX_STANDARD 14)

# Encontra os pacotes necessários para o seu pacote
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Adiciona o arquivo fonte C++
add_executable(llm_service_node src/llm_service_node.cpp)

# Linkar o executável com a biblioteca do ROS 2 (rclcpp, std_srvs, etc.)
ament_target_dependencies(llm_service_node
  rclcpp
  std_srvs
)

# Instalar o executável do C++ no diretório bin
install(TARGETS
  llm_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar os scripts Python
install(PROGRAMS
  scripts/llm_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar os arquivos de cabeçalhos C++ se necessário
install(DIRECTORY include/
  DESTINATION include/
)

# Instalar os arquivos de launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
# Exporta o pacote ROS 2
ament_package()

