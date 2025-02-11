if(NOT DEFINED USE_ROS2)
  find_package(ament_cmake_gtest REQUIRED)
  include(GoogleTest)
  # Prevents discovery failure before install
  # No effect in CMake < 3.18
  set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
  function(add_CCC_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest CCC)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  find_package(ament_cmake_gtest REQUIRED)
  function(add_CCC_test NAME)
    ament_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} CCC)
  endfunction()
endif()

set(CCC_gtest_list
  TestStateSpaceModel
  TestVariantSequentialExtension
  TestInvariantSequentialExtension
  TestPreviewControlZmp
  TestDdpZmp
  TestDcmTracking
  TestFootGuidedControl
  TestLinearMpcZmp
  TestIntrinsicallyStableMpc
  TestSingularPreviewControlZmp
  TestStepMpc
  TestLinearMpcZ
  TestLinearMpcXY
  TestPreviewControlCentroidal
  TestDdpCentroidal
  TestDdpSingleRigidBody
  )

foreach(NAME IN LISTS CCC_gtest_list)
  add_CCC_test(${NAME})
endforeach()

option(ENABLE_PYBULLET_TEST "Enable tests with pybullet simulation" OFF)
if(ENABLE_PYBULLET_TEST)
  set(CCC_rostest_list
    TestSimDdpSingleRigidBody
  )

  if(DEFINED USE_ROS2)
    foreach(NAME IN LISTS CCC_rostest_list)
      ament_add_gtest(${NAME} test/${NAME}.test src/${NAME}.cpp)
      target_link_libraries(${NAME} CCC)
    endforeach()
  endif()
endif()
