cmake_minimum_required(VERSION 3.8)
project(robotarm_rbd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

# ROS free rigid body model (RobotarmRbd): URDF parsing via urdfdom and the calculations, Eigen only.
# Kept as its own library so nothing from rclcpp can slip in unnoticed.
add_library(${PROJECT_NAME}_core SHARED
  src/RobotarmRbd.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  urdfdom::urdfdom_model
  Eigen3::Eigen
)

add_library(${PROJECT_NAME} SHARED
  src/Kinematics.cpp
)

# At least C++17 (Eigen fixed-size members in std::vector rely on aligned new)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  kinematics_interface::kinematics_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  urdf::urdf
  Eigen3::Eigen
)

pluginlib_export_plugin_description_file(kinematics_interface kinematics_plugins.xml)

# Tools, not part of the library: Cartesian jogging (twists on cmd_vel -> /joint_states), see
# tools/cartesian_jog/ and its launch file. The kinematics plugin is loaded through pluginlib; the
# library is linked only to reach the robot model getters of Kinematics (joint names, limits, tcp).
# Nothing under tools/ is installed as a header.
add_executable(cartesian_jog tools/cartesian_jog/cartesian_jog_node.cpp)
target_compile_features(cartesian_jog PUBLIC cxx_std_17)
target_link_libraries(cartesian_jog
  ${PROJECT_NAME}
  kinematics_interface::kinematics_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

install(
  TARGETS cartesian_jog
  DESTINATION lib/${PROJECT_NAME}
)

# six sliders for the twist on cmd_vel, the default input of cartesian_jog.launch.py
install(
  PROGRAMS tools/cartesian_jog/cartesian_jog_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cartesian_jog_gui
)

install(
  FILES tools/cartesian_jog/cartesian_jog.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}
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
  urdfdom
  eigen3_cmake_module
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  # URDF parsing, kinematics and conversions against independent references. The tests have their
  # own main() (rclcpp::init), so the gtest main library is not linked.
  ament_add_gtest(kinematics_test test/kinematics_test.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_include_directories(kinematics_test PRIVATE test)
  target_link_libraries(kinematics_test
    ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
    urdf::urdf
    Eigen3::Eigen
  )

  # RobotarmRbd, linked without rclcpp so the test fails to link if the library depends on ROS
  ament_add_gtest(robotarm_rbd_test test/robotarm_rbd_test.cpp)
  target_include_directories(robotarm_rbd_test PRIVATE test)
  target_link_libraries(robotarm_rbd_test ${PROJECT_NAME}_core)

  # joint limit policy of the jog tool (pure function, no ROS needed)
  ament_add_gtest(joint_limiter_test tools/cartesian_jog/joint_limiter_test.cpp)
  target_link_libraries(joint_limiter_test Eigen3::Eigen)

  # Allocation check of the rt path. Eigen's malloc guard has to be compiled into the code under
  # test, so Kinematics.cpp and RobotarmRbd.cpp (where the rt code lives) are compiled into this
  # executable (instead of linking the libraries) with test/eigen_malloc_guard.hpp force-included.
  ament_add_gtest(kinematics_malloc_test
    test/kinematics_malloc_test.cpp
    src/Kinematics.cpp
    src/RobotarmRbd.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_compile_options(kinematics_malloc_test PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/test/eigen_malloc_guard.hpp)
  target_include_directories(kinematics_malloc_test PRIVATE include test)
  target_compile_features(kinematics_malloc_test PUBLIC cxx_std_17)
  target_link_libraries(kinematics_malloc_test
    urdfdom::urdfdom_model
    kinematics_interface::kinematics_interface
    pluginlib::pluginlib
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
    urdf::urdf
    Eigen3::Eigen
  )
endif()

ament_package()