cmake_minimum_required(VERSION 3.8)
project(kortex_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros2_interfaces REQUIRED)

# Kortex API paths (local to this package)
set(KORTEX_API_DIR "${CMAKE_CURRENT_SOURCE_DIR}/kortex_api")
set(KORTEX_INCLUDE_DIR "${KORTEX_API_DIR}/include")
set(KORTEX_LIB_RELEASE "${KORTEX_API_DIR}/lib/release/libKortexApiCpp.a")

# Verify Kortex API exists
if(NOT EXISTS ${KORTEX_INCLUDE_DIR})
    message(FATAL_ERROR "Kortex API headers not found at ${KORTEX_INCLUDE_DIR}.")
endif()

# Include directories - You MUST include the subfolders for the compiler to find the .h files
include_directories(include)
include_directories(${KORTEX_INCLUDE_DIR})
include_directories(${KORTEX_INCLUDE_DIR}/client)
include_directories(${KORTEX_INCLUDE_DIR}/client_stubs)
include_directories(${KORTEX_INCLUDE_DIR}/common)
include_directories(${KORTEX_INCLUDE_DIR}/messages)

include_directories(include ${KORTEX_INCLUDE_DIR})

add_executable(controller src/controller.cpp)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(controller
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  ros2_interfaces
)

target_link_libraries(controller
  ${KORTEX_LIB_RELEASE}
  pthread
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
