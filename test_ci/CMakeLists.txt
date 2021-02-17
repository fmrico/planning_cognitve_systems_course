cmake_minimum_required(VERSION 3.5)
project(test_ci)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/test_ci/TestNode.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(main_program
  src/main_program.cpp
)
ament_target_dependencies(main_program ${dependencies})
target_link_libraries(main_program ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  main_program
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_dependencies(${dependencies})
ament_package()
