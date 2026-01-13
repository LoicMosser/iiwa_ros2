cmake_minimum_required(VERSION 3.8)
project(iiwa_velocity_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(std_msgs REQUIRED)

# Ajoutez cette ligne pour inclure automatiquement le répertoire 'include'
include_directories(include)

add_executable(iiwa_velocity_publisher src/iiwa_velocity_publisher.cpp)

# Spécifiez explicitement le chemin d'inclusion pour la cible
target_include_directories(iiwa_velocity_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(iiwa_velocity_publisher
  std_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  orocos_kdl
  kdl_parser
  urdf
  urdfdom_headers
)

install(TARGETS iiwa_velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
