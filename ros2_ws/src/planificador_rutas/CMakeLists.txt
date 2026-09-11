cmake_minimum_required(VERSION 3.8)
project(planificador_rutas)

# Opciones de compilación (estándar de ROS 2)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- BUSCAR DEPENDENCIAS ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED) # Necesario para <nlohmann/json.hpp>

# --- INCLUIR CABECERAS ---
# Asume que tu .hpp está dentro de la carpeta 'include/'
include_directories(include)

# --- DEFINIR EJECUTABLE ---
# Asume que tu .cpp está en 'src/planificador.cpp'
add_executable(planificador_node
  src/planificador.cpp
)

# --- VINCULAR DEPENDENCIAS ---
ament_target_dependencies(planificador_node
  rclcpp
  geometry_msgs
  std_msgs
  nlohmann_json
)

# --- INSTALACIÓN ---
# Esto copia el ejecutable a la carpeta donde ROS puede ejecutarlo (ros2 run...)
install(TARGETS planificador_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar los headers (opcional, pero buena práctica si otros nodos usan tu librería)
install(DIRECTORY include/
  DESTINATION include
)

# --- TESTING (Tu configuración original) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()