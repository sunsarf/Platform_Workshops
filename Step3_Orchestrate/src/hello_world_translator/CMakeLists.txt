cmake_minimum_required(VERSION 3.8)
project(hello_world_translator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create libraries
add_library(TranslatorLib src/translator_node.cpp)
ament_target_dependencies(TranslatorLib rclcpp std_msgs)

# Include directories
target_include_directories(TranslatorLib PUBLIC include/hello_world_translator)

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

# Define executables in the package
add_executable(translate src/main.cpp)
ament_target_dependencies(translate rclcpp)
target_include_directories(translate PRIVATE include/hello_world_translator)
target_link_libraries(translate TranslatorLib)

# Install cpp executables 
install(
  TARGETS
  translate 
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
