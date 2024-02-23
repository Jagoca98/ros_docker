cmake_minimum_required(VERSION 3.5)
project(cov_mat)

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

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

# find system dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PCL 1.7 REQUIRED COMPONENTS common visualization)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(${PROJECT_NAME}_lib
  src/lib/anova.cpp
  src/lib/fileBridge.cpp
  src/lib/hungarian.cpp
  src/lib/truthMatcher.cpp
  src/lib/detectionDrawer.cpp
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node
                      ${PROJECT_NAME}_lib
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
)

# Install C++ executables
install(TARGETS
	${PROJECT_NAME}_node
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
