cmake_minimum_required(VERSION 3.14)
project(my_project)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  hello_test
  hello_test.cc
)
target_link_libraries(
  hello_test
  GTest::gtest_main
)

add_executable(
  command_test
  command_test.cc
)
  target_link_libraries(
    command_test
    GTest::gtest_main
    dxl_x64_cpp
    rt
)

include(GoogleTest)
gtest_discover_tests(hello_test)
gtest_discover_tests(command_test)

# 添加包含路径
include_directories(${CMAKE_SOURCE_DIR}/c++/include/dynamixel_sdk)
