cmake_minimum_required(VERSION 3.8)
project(robotarm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/KinematicsCore.cpp
  src/Kinematics.cpp
)

# At least C++17 (Eigen fixed-size members in std::vector rely on aligned new)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  kinematics_interface::kinematics_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  urdf::urdf
  Eigen3::Eigen
)

pluginlib_export_plugin_description_file(kinematics_interface kinematics_plugins.xml)

# manual check: parses the URDF and prints the joints / DH params, see launch/
add_executable(kinematics_core_test test/kinematics_core_test.cpp)
target_link_libraries(kinematics_core_test
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

install(
  TARGETS kinematics_core_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES kinematics_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  kinematics_interface
  pluginlib
  rclcpp
  urdf
  eigen3_cmake_module
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()