cmake_minimum_required(VERSION 3.8)
project(hacka_analaura)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(dependencies 
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs 
)

include_directories(
  include
)

set(library_name hacka_analaura_library)
set(executable_name hacka_analaura)

# Criar a biblioteca do nó
add_library(${library_name} SHARED
  src/mission_main.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

rclcpp_components_register_nodes(${library_name} "hacka_analaura::MissionNode")

# Criar o executável
add_executable(${executable_name}
  src/mission_main.cpp
)

target_link_libraries(${executable_name} ${library_name})

target_include_directories(${library_name} PRIVATE include)

# Instalar a biblioteca e o executável
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Instalar os diretórios necessários
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

# Configuração de testes
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
