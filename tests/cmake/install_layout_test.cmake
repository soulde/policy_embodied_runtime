if(NOT DEFINED POLICY_RUNTIME_BUILD_DIR OR
   NOT DEFINED POLICY_RUNTIME_INSTALL_LIBDIR OR
   NOT DEFINED POLICY_RUNTIME_INSTALL_TEST_ROOT)
    message(FATAL_ERROR "install layout test requires build, libdir, and root")
endif()

file(REMOVE_RECURSE "${POLICY_RUNTIME_INSTALL_TEST_ROOT}")
file(MAKE_DIRECTORY "${POLICY_RUNTIME_INSTALL_TEST_ROOT}")
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env
        "DESTDIR=${POLICY_RUNTIME_INSTALL_TEST_ROOT}"
        "${CMAKE_COMMAND}" --install "${POLICY_RUNTIME_BUILD_DIR}"
        --prefix /usr
    RESULT_VARIABLE install_result
    OUTPUT_VARIABLE install_output
    ERROR_VARIABLE install_error)
if(NOT install_result EQUAL 0)
    message(FATAL_ERROR
        "DESTDIR install failed (${install_result}):\n"
        "${install_output}${install_error}")
endif()

set(expected_unit
    "${POLICY_RUNTIME_INSTALL_TEST_ROOT}/usr/lib/systemd/system/robot-io-daemon.service")
if(NOT EXISTS "${expected_unit}")
    message(FATAL_ERROR
        "systemd unit was not installed at /usr/lib/systemd/system")
endif()

set(gnu_libdir_unit
    "${POLICY_RUNTIME_INSTALL_TEST_ROOT}/usr/${POLICY_RUNTIME_INSTALL_LIBDIR}/systemd/system/robot-io-daemon.service")
if(NOT gnu_libdir_unit STREQUAL expected_unit AND
   EXISTS "${gnu_libdir_unit}")
    message(FATAL_ERROR
        "systemd unit was incorrectly installed below GNUInstallDirs LIBDIR")
endif()
