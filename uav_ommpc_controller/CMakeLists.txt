cmake_minimum_required(VERSION 3.16)
project(uav_ommpc_controller VERSION 0.1.0 LANGUAGES C CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include(cmake/FetchOsqp.cmake)

add_library(${PROJECT_NAME} SHARED src/ommpc.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen PRIVATE osqpstatic ${CMAKE_DL_LIBS})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

option(BUILD_EXAMPLES "Build the nonlinear OMMPC tracking benchmark" ON)
if(BUILD_EXAMPLES)
  add_executable(ommpc_tracking examples/ommpc_tracking.cpp)
  target_link_libraries(ommpc_tracking PRIVATE ${PROJECT_NAME})
  set_target_properties(ommpc_tracking PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  install(TARGETS ommpc_tracking RUNTIME DESTINATION bin)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_so3_math test/test_so3_math.cpp)
  target_link_libraries(test_so3_math PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_executable(test_dynamic test/test_dynamic.cpp)
  target_link_libraries(test_dynamic PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_executable(test_linearization test/test_linearization.cpp)
  target_link_libraries(test_linearization PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_executable(test_ommpc test/test_ommpc.cpp)
  target_link_libraries(test_ommpc PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_so3_math PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(test_dynamic PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(test_linearization PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(test_ommpc PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  include(GoogleTest)
  gtest_discover_tests(test_so3_math)
  gtest_discover_tests(test_dynamic)
  gtest_discover_tests(test_linearization)
  gtest_discover_tests(test_ommpc)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/uav_ommpc_controllerConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(EXPORT ${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(FILES ${osqp_SOURCE_DIR}/LICENSE ${osqp_SOURCE_DIR}/NOTICE
  DESTINATION share/${PROJECT_NAME}/licenses/osqp)
install(FILES ${osqp_SOURCE_DIR}/lin_sys/direct/qdldl/qdldl_sources/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses/qdldl)
install(FILES ${osqp_SOURCE_DIR}/lin_sys/direct/qdldl/amd/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses/amd)
