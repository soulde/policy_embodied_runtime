cmake_minimum_required(VERSION 3.24)

project(policy_runtime_cyclonedds_probe LANGUAGES CXX)

if(NOT DEFINED POLICY_RUNTIME_SOURCE_DIR)
    message(FATAL_ERROR "POLICY_RUNTIME_SOURCE_DIR is required")
endif()

list(APPEND CMAKE_MODULE_PATH "${POLICY_RUNTIME_SOURCE_DIR}/cmake")
include(PolicyRuntimeDds)

if(NOT POLICY_RUNTIME_HAS_DDS)
    message(FATAL_ERROR "expected Cyclone DDS support to be enabled")
endif()
if(NOT POLICY_RUNTIME_DDS_C_TARGET STREQUAL "CycloneDDS::ddsc")
    message(FATAL_ERROR "expected the Cyclone DDS C package target")
endif()
if(NOT POLICY_RUNTIME_DDS_CXX_TARGET STREQUAL "CycloneDDS-CXX::ddscxx")
    message(FATAL_ERROR "expected the Cyclone DDS C++ package target")
endif()

policy_runtime_add_dds_types(
    TARGET cyclonedds_probe_types
    IDL "${CMAKE_CURRENT_SOURCE_DIR}/probe.idl")
if(NOT TARGET cyclonedds_probe_types)
    message(FATAL_ERROR "expected the Cyclone DDS C++ IDL target")
endif()
get_property(CYCLONEDDS_PROBE_IDL TARGET cyclonedds_probe_types
    PROPERTY POLICY_RUNTIME_FAKE_DDS_IDL)
if(NOT CYCLONEDDS_PROBE_IDL STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}/probe.idl")
    message(FATAL_ERROR "expected the Cyclone DDS C++ IDL generator to run")
endif()
