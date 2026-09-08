if(NOT TARGET CycloneDDS-CXX::ddscxx)
    add_library(CycloneDDS-CXX::ddscxx INTERFACE IMPORTED)
endif()

function(idlcxx_generate)
    set(one_value_arguments TARGET)
    set(multi_value_arguments FILES)
    cmake_parse_arguments(IDLCXX "" "${one_value_arguments}"
        "${multi_value_arguments}" ${ARGN})

    if(NOT IDLCXX_TARGET OR NOT IDLCXX_FILES)
        message(FATAL_ERROR "fake idlcxx_generate requires TARGET and FILES")
    endif()
    add_library("${IDLCXX_TARGET}" INTERFACE)
    set_property(TARGET "${IDLCXX_TARGET}" PROPERTY
        POLICY_RUNTIME_FAKE_DDS_IDL "${IDLCXX_FILES}")
endfunction()
