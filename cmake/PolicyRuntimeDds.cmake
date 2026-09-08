option(POLICY_RUNTIME_WITH_DDS "Build Cyclone DDS robot I/O IPC" OFF)

set(POLICY_RUNTIME_HAS_DDS OFF)
set(POLICY_RUNTIME_DDS_C_TARGET "")
set(POLICY_RUNTIME_DDS_CXX_TARGET "")

if(POLICY_RUNTIME_WITH_DDS)
    find_package(CycloneDDS CONFIG REQUIRED)
    find_package(CycloneDDS-CXX CONFIG REQUIRED)

    if(NOT TARGET CycloneDDS::ddsc)
        message(FATAL_ERROR
            "CycloneDDS must export the CycloneDDS::ddsc target")
    endif()
    if(NOT TARGET CycloneDDS-CXX::ddscxx)
        message(FATAL_ERROR
            "CycloneDDS-CXX must export the CycloneDDS-CXX::ddscxx target")
    endif()
    if(NOT COMMAND idlcxx_generate)
        message(FATAL_ERROR
            "CycloneDDS-CXX must export the idlcxx_generate command")
    endif()

    set(POLICY_RUNTIME_DDS_C_TARGET CycloneDDS::ddsc)
    set(POLICY_RUNTIME_DDS_CXX_TARGET CycloneDDS-CXX::ddscxx)
    set(POLICY_RUNTIME_HAS_DDS ON)
endif()

function(policy_runtime_add_dds_types)
    set(one_value_arguments TARGET IDL)
    cmake_parse_arguments(DDS "" "${one_value_arguments}" "" ${ARGN})

    if(DDS_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR
            "policy_runtime_add_dds_types received unexpected arguments: "
            "${DDS_UNPARSED_ARGUMENTS}")
    endif()
    if(NOT DDS_TARGET OR NOT DDS_IDL)
        message(FATAL_ERROR
            "policy_runtime_add_dds_types requires TARGET and IDL")
    endif()
    if(NOT POLICY_RUNTIME_HAS_DDS)
        message(FATAL_ERROR
            "policy_runtime_add_dds_types requires POLICY_RUNTIME_WITH_DDS=ON")
    endif()

    idlcxx_generate(TARGET "${DDS_TARGET}" FILES "${DDS_IDL}")
endfunction()
