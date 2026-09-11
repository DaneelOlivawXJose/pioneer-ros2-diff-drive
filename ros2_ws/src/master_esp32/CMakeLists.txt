cmake_minimum_required(VERSION 3.8)
project(master_esp32)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. Incluir directorios de cabecera (.hpp)
include_directories(include)

# 3. Crear el ejecutable
# "master_node" es el nombre que usarás en ros2 run (ros2 run master_esp32 master_node)
# "src/master.cpp" es tu archivo de código fuente
add_executable(master_node src/master.cpp)

# 4. Vincular dependencias al ejecutable
ament_target_dependencies(master_node
  rclcpp
  std_msgs
  geometry_msgs
)

# 5. Instalar el ejecutable (para que ROS 2 lo encuentre)
install(TARGETS
  master_node
  DESTINATION lib/${PROJECT_NAME}
)

# 6. Instalar headers (buena práctica)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()