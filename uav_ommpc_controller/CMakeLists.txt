cmake_minimum_required(VERSION 3.16)
project(uav_ommpc_controller VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_so3_math test/test_so3_math.cpp)
  target_link_libraries(test_so3_math PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_executable(test_dynamic test/test_dynamic.cpp)
  target_link_libraries(test_dynamic PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_so3_math PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(test_dynamic PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  include(GoogleTest)
  gtest_discover_tests(test_so3_math)
  gtest_discover_tests(test_dynamic)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/uav_ommpc_controllerConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(EXPORT ${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
