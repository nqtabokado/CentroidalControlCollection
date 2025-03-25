cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME centroidal_control_collection)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/CentroidalControlCollection)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(centroidal_control_collection LANGUAGES CXX)

if(DEFINED USE_ROS2)
  set(DOXYGEN_HTML_OUTPUT html)

  # mc_rtc
  add_project_dependency(mc_rtc REQUIRED)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(qp_solver_collection REQUIRED)
  find_package(force_control_collection REQUIRED)
  find_package(nmpc_ddp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  ament_export_dependencies(
    rclcpp
    qp_solver_collection
    force_control_collection
    nmpc_ddp
    EIGEN3
    std_msgs
    std_srvs
    visualization_msgs
  )

else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
  set(BUILD_TESTING OFF)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
  add_project_dependency(qp_solver_collection REQUIRED)
  add_project_dependency(force_control_collection REQUIRED)
  add_project_dependency(nmpc_ddp REQUIRED)
endif()

add_subdirectory(src)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

if(USE_ROS2)
  ament_package()
endif()
