cmake_minimum_required(VERSION 3.8)
project(interfaz_hardware_omni_v1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

# Incluir directorios de encabezados
include_directories(
  include
)

# Compilar la interfaz de hardware como una biblioteca compartida para pluginlib
add_library(${PROJECT_NAME}_hardware_interface SHARED
  src/hardware_interface.cpp
)

# Directorios de encabezados para la interfaz de hardware
target_include_directories(${PROJECT_NAME}_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Configurar características de compilación para la biblioteca
target_compile_features(${PROJECT_NAME}_hardware_interface PUBLIC c_std_99 cxx_std_17)

# Dependencias de la biblioteca
ament_target_dependencies(${PROJECT_NAME}_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  geometry_msgs
  std_msgs
  sensor_msgs
  serial
)

# Exportar el plugin de la interfaz de hardware para pluginlib
pluginlib_export_plugin_description_file(hardware_interface omni_robot_hardware_interface.xml)

# Instalación de la biblioteca y archivos de configuración
install(TARGETS ${PROJECT_NAME}_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar los archivos de encabezado y el archivo XML de pluginlib
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

install(FILES omni_robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos de configuración (como config.yaml)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

# Pruebas y linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
