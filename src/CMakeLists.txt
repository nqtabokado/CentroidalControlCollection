if(USE_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(visualization_msgs REQUIRED)
endif()

add_library(CCC
  CommonModels.cpp
  PreviewControlZmp.cpp
  DdpZmp.cpp
  DcmTracking.cpp
  FootGuidedControl.cpp
  LinearMpcZmp.cpp
  IntrinsicallyStableMpc.cpp
  SingularPreviewControlZmp.cpp
  StepMpc.cpp
  LinearMpcZ.cpp
  LinearMpcXY.cpp
  PreviewControlCentroidal.cpp
  DdpCentroidal.cpp
  DdpSingleRigidBody.cpp
  )
if(DEFINED USE_ROS2)
  target_include_directories(CCC PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(CCC PUBLIC
    mc_rtc::mc_rtc_utils mc_rtc::mc_rtc_gui qp_solver_collection::QpSolverCollection force_control_collection::ForceColl nmpc_ddp::nmpc_ddp
  )
else()
  target_link_libraries(CCC PUBLIC
    qp_solver_collection::QpSolverCollection
    force_control_collection::ForceColl
    nmpc_ddp::nmpc_ddp
  )
  target_compile_definitions(CCC PUBLIC CCC_STANDALONE)
endif()
target_include_directories(CCC PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(CCC PUBLIC cxx_std_17)

ament_target_dependencies(CCC PUBLIC
  rclcpp
  std_msgs
  std_srvs
  visualization_msgs
)

if(BUILD_SHARED_LIBS)
  set_target_properties(CCC PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
endif()

install(TARGETS CCC
          EXPORT "${TARGETS_EXPORT_NAME}"
          LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
          ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
          RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/CCC DESTINATION "${INCLUDE_INSTALL_DIR}")
