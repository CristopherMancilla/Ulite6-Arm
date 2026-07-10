cmake_minimum_required(VERSION 3.8)
project(ulite6_arm_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to c++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf_tutorial REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ====================================================================
# REGLA DE INSTALACIÓN CORREGIDA
# ====================================================================
# Al copiar las carpetas raíz, CMake mantendrá la estructura idéntica:
# share/ulite6_arm_description/urdf/mech/archivo.xacro
install(DIRECTORY
  launch 
  meshes
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()