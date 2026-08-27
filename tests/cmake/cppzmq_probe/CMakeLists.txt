cmake_minimum_required(VERSION 3.24)

project(policy_runtime_cppzmq_probe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if(NOT DEFINED POLICY_RUNTIME_SOURCE_DIR)
    message(FATAL_ERROR "POLICY_RUNTIME_SOURCE_DIR is required")
endif()
if(NOT DEFINED CPPZMQ_INCLUDE_DIR)
    message(FATAL_ERROR "CPPZMQ_INCLUDE_DIR is required")
endif()
if(NOT DEFINED EXPECT_SUPPORTED)
    message(FATAL_ERROR "EXPECT_SUPPORTED is required")
endif()
if(NOT DEFINED PROBE_MODE)
    set(PROBE_MODE INCLUDE)
endif()

list(APPEND CMAKE_MODULE_PATH "${POLICY_RUNTIME_SOURCE_DIR}/cmake")
include(PolicyRuntimeZmq)

if(PROBE_MODE STREQUAL "TARGET")
    add_library(fake_cppzmq INTERFACE IMPORTED)
    set_target_properties(fake_cppzmq PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CPPZMQ_INCLUDE_DIR}")
    policy_runtime_check_cppzmq_api(
        CPPZMQ_API_SUPPORTED
        TARGET fake_cppzmq)
elseif(PROBE_MODE STREQUAL "INCLUDE")
    policy_runtime_check_cppzmq_api(
        CPPZMQ_API_SUPPORTED
        INCLUDE_DIR "${CPPZMQ_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "PROBE_MODE must be INCLUDE or TARGET")
endif()

if(EXPECT_SUPPORTED AND NOT CPPZMQ_API_SUPPORTED)
    message(FATAL_ERROR "expected the cppzmq API probe to succeed")
endif()
if(NOT EXPECT_SUPPORTED AND CPPZMQ_API_SUPPORTED)
    message(FATAL_ERROR "expected the cppzmq API probe to fail")
endif()
