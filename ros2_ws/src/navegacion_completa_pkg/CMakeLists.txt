cmake_minimum_required(VERSION 3.8)
project(navegacion_completa_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencias base
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # <-- NUEVO: Necesario para los Twist y Point

# 2. Dependencias para generar Mensajes/Acciones
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# 3. Generar la Acción (Asegúrate de que la carpeta se llama 'action')
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToNode.action"
  DEPENDENCIES action_msgs
)

# 4. Crear los ejecutables
add_executable(nav_manager src/nav_manager.cpp)
add_executable(pure_pursuit src/pure_pursuit.cpp) # <-- NUEVO: Tu nodo de seguimiento
add_executable(mpc_controller src/mpc.cpp)

# 5. Añadir directorios de includes
target_include_directories(nav_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(pure_pursuit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) # <-- NUEVO: Para que encuentre pure_pursuit.hpp

target_include_directories(mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 6. Vincular dependencias de ROS 2
ament_target_dependencies(nav_manager rclcpp rclcpp_action std_msgs)
ament_target_dependencies(pure_pursuit rclcpp std_msgs geometry_msgs) # <-- NUEVO: Sus librerías
ament_target_dependencies(mpc_controller
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(mpc_controller
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

# 8. Instalar los binarios compilados
install(TARGETS 
  nav_manager
  pure_pursuit # <-- NUEVO: Si no lo pones aquí, ros2 run no lo encontrará
  mpc_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()