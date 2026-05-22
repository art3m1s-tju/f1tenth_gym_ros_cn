cmake_minimum_required(VERSION 3.8)
project(f1tenth_gym_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(lqr_core STATIC src/lqr_core.c)
target_include_directories(lqr_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(lqr_core m)

add_executable(lqr_controller src/lqr_controller_node.cpp)
target_include_directories(lqr_controller PRIVATE include)
target_link_libraries(lqr_controller lqr_core)
ament_target_dependencies(
  lqr_controller
  ackermann_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros)

install(TARGETS lqr_controller
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

ament_python_install_package(${PROJECT_NAME})

set(PY_EXECUTABLES
  gym_bridge)

foreach(executable ${PY_EXECUTABLES})
  set(wrapper "${CMAKE_CURRENT_BINARY_DIR}/${executable}")
  file(WRITE "${wrapper}"
    "#!/usr/bin/env python3\n"
    "from ${PROJECT_NAME}.${executable} import main\n"
    "\n"
    "if __name__ == '__main__':\n"
    "    main()\n")
  install(PROGRAMS "${wrapper}" DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

ament_package()
