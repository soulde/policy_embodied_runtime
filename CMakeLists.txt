cmake_minimum_required(VERSION 3.24)

project(policy_runtime LANGUAGES CXX)

option(POLICY_RUNTIME_BUILD_TESTS "Build policy runtime unit tests" ON)

include(cmake/ProjectOptions.cmake)
include(FetchContent)

find_package(nlohmann_json 3.11.3 CONFIG QUIET)
if(NOT TARGET nlohmann_json::nlohmann_json)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
        URL_HASH SHA256=0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

add_library(policy_runtime_core
    src/common/result.cpp
    src/profiles/loader.cpp
    src/protocol/rpc/codec.cpp
    src/robot_io/transport_scheduler.cpp)
target_include_directories(policy_runtime_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(policy_runtime_core PUBLIC nlohmann_json::nlohmann_json)
policy_runtime_configure_target(policy_runtime_core)

if(POLICY_RUNTIME_BUILD_TESTS)
    include(CTest)
    enable_testing()

    find_package(GTest CONFIG QUIET)
    if(NOT TARGET GTest::gtest)
        find_package(GTest QUIET)
    endif()

    if(NOT TARGET GTest::gtest)
        FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
            URL_HASH SHA256=8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7)
        FetchContent_MakeAvailable(googletest)
    endif()

    add_executable(policy_runtime_unit_tests tests/cpp/unit/result_test.cpp)
    target_link_libraries(policy_runtime_unit_tests
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(policy_runtime_unit_tests)

    add_executable(rpc_codec_test tests/cpp/unit/rpc_codec_test.cpp)
    target_link_libraries(rpc_codec_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    target_compile_definitions(rpc_codec_test
        PRIVATE POLICY_RUNTIME_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    policy_runtime_configure_target(rpc_codec_test)

    add_executable(profile_loader_test tests/cpp/unit/profile_loader_test.cpp)
    target_link_libraries(profile_loader_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    target_compile_definitions(profile_loader_test
        PRIVATE POLICY_RUNTIME_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    policy_runtime_configure_target(profile_loader_test)

    add_executable(transport_scheduler_test tests/cpp/unit/transport_scheduler_test.cpp)
    target_link_libraries(transport_scheduler_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(transport_scheduler_test)

    include(GoogleTest)
    gtest_discover_tests(policy_runtime_unit_tests TEST_PREFIX result_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(rpc_codec_test TEST_PREFIX rpc_codec_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(profile_loader_test TEST_PREFIX profile_loader_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(transport_scheduler_test TEST_PREFIX transport_scheduler_test.
        DISCOVERY_MODE PRE_TEST)
endif()
