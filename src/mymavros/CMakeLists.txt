cmake_minimum_required(VERSION 3.8)
project(mymavros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(mymavros src/takeoffvs.cpp)
add_executable(arm src/bir.cpp)
add_executable(takeoff src/iki.cpp)
add_executable(uc src/uc.cpp)

ament_target_dependencies(mymavros
  rclcpp
  mavros_msgs
)
ament_target_dependencies(arm
  rclcpp
  mavros_msgs
)
ament_target_dependencies(takeoff
  rclcpp
  mavros_msgs
)

ament_target_dependencies(uc
  rclcpp
  mavros_msgs
)



install(TARGETS
   mymavros
   arm
   takeoff
   uc
   DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
