cmake_minimum_required(VERSION 3.24)

project(policy_runtime LANGUAGES CXX)

option(POLICY_RUNTIME_BUILD_TESTS "Build policy runtime unit tests" ON)
option(POLICY_RUNTIME_WITH_IGH "Build the IgH EtherCAT backend" OFF)
option(POLICY_RUNTIME_WITH_ZMQ "Build the ZeroMQ policy RPC server when available" ON)
set(POLICY_RUNTIME_SYSTEMD_UNIT_DIR "lib/systemd/system" CACHE PATH
    "Install directory for systemd system units")

include(cmake/ProjectOptions.cmake)
include(FetchContent)
include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(PolicyRuntimeZmq)

if(POLICY_RUNTIME_WITH_IGH)
    find_package(EtherCAT REQUIRED)
endif()

set(POLICY_RUNTIME_HAS_ZMQ OFF)
if(POLICY_RUNTIME_WITH_ZMQ)
    find_package(cppzmq CONFIG QUIET)
    if(TARGET cppzmq)
        set(POLICY_RUNTIME_CPPZMQ_PACKAGE_TARGET cppzmq)
    elseif(TARGET cppzmq-static)
        set(POLICY_RUNTIME_CPPZMQ_PACKAGE_TARGET cppzmq-static)
    endif()
    if(POLICY_RUNTIME_CPPZMQ_PACKAGE_TARGET)
        policy_runtime_check_cppzmq_api(
            POLICY_RUNTIME_CPPZMQ_PACKAGE_API_SUPPORTED
            TARGET "${POLICY_RUNTIME_CPPZMQ_PACKAGE_TARGET}")
        if(POLICY_RUNTIME_CPPZMQ_PACKAGE_API_SUPPORTED)
            set(POLICY_RUNTIME_ZMQ_TARGET
                "${POLICY_RUNTIME_CPPZMQ_PACKAGE_TARGET}")
            set(POLICY_RUNTIME_HAS_ZMQ ON)
        endif()
    endif()
    if(NOT POLICY_RUNTIME_HAS_ZMQ)
        find_path(POLICY_RUNTIME_CPPZMQ_INCLUDE_DIR zmq.hpp)
        find_library(POLICY_RUNTIME_ZMQ_LIBRARY NAMES zmq)
        if(POLICY_RUNTIME_CPPZMQ_INCLUDE_DIR AND POLICY_RUNTIME_ZMQ_LIBRARY)
            policy_runtime_check_cppzmq_api(
                POLICY_RUNTIME_CPPZMQ_FALLBACK_API_SUPPORTED
                INCLUDE_DIR "${POLICY_RUNTIME_CPPZMQ_INCLUDE_DIR}")
            if(POLICY_RUNTIME_CPPZMQ_FALLBACK_API_SUPPORTED)
                add_library(policy_runtime_zmq INTERFACE)
                target_include_directories(policy_runtime_zmq INTERFACE
                    "${POLICY_RUNTIME_CPPZMQ_INCLUDE_DIR}")
                target_link_libraries(policy_runtime_zmq INTERFACE
                    "${POLICY_RUNTIME_ZMQ_LIBRARY}")
                set(POLICY_RUNTIME_ZMQ_TARGET policy_runtime_zmq)
                set(POLICY_RUNTIME_HAS_ZMQ ON)
            endif()
        endif()
    endif()
endif()
if(POLICY_RUNTIME_HAS_ZMQ)
    message(STATUS "Building policy-runtime-host with ZeroMQ/cppzmq support")
else()
    message(STATUS "Compatible ZeroMQ/cppzmq development files not found; building direct runtime host only")
endif()

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
    src/policy/dummy_policy.cpp
    src/policy/pipeline.cpp
    src/profiles/loader.cpp
    src/protocol/cia402/state_machine.cpp
    src/protocol/damiao/protocol.cpp
    src/protocol/st3215/protocol.cpp
    src/protocol/rpc/codec.cpp
    src/robot/devices/cia402/axis.cpp
    src/robot/devices/st3215/servo.cpp
    src/robot_io/daemon.cpp
    src/robot_io/ipc_server.cpp
    src/robot_io/realtime_loop.cpp
    src/robot_io/safety_supervisor.cpp
    src/robot_io/service_listener.cpp
    src/runtime/runtime_host.cpp
    src/runtime/runtime_host_cli.cpp
    src/runtime/robot_io_client.cpp
    src/runtime/robot_io_service.cpp
    src/robot_io/transport_scheduler.cpp
    src/transport/ethercat/elmo_gold.cpp
    src/transport/ethercat/master.cpp
    src/transport/serial/serial_transport.cpp
    src/transport/serial/virtual_can_transport.cpp
    src/transport/socketcan/socketcan_transport.cpp)
target_include_directories(policy_runtime_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(policy_runtime_core PUBLIC nlohmann_json::nlohmann_json)
find_package(Threads REQUIRED)
target_link_libraries(policy_runtime_core PUBLIC Threads::Threads)
target_compile_definitions(policy_runtime_core
    PUBLIC POLICY_RUNTIME_WITH_IGH=$<BOOL:${POLICY_RUNTIME_WITH_IGH}>)
if(POLICY_RUNTIME_WITH_IGH)
    target_sources(policy_runtime_core PRIVATE src/transport/ethercat/igh_backend.cpp)
    target_link_libraries(policy_runtime_core PRIVATE EtherCAT::EtherCAT)
endif()
policy_runtime_configure_target(policy_runtime_core)

add_executable(robot-io-daemon apps/robot_io_daemon_main.cpp)
target_link_libraries(robot-io-daemon PRIVATE policy_runtime_core)
policy_runtime_configure_target(robot-io-daemon)

add_executable(policy-runtime-host apps/policy_runtime_host_main.cpp)
target_link_libraries(policy-runtime-host PRIVATE policy_runtime_core)
target_compile_definitions(policy-runtime-host PRIVATE
    POLICY_RUNTIME_HAS_ZMQ=$<BOOL:${POLICY_RUNTIME_HAS_ZMQ}>)
if(POLICY_RUNTIME_HAS_ZMQ)
    target_link_libraries(policy-runtime-host PRIVATE ${POLICY_RUNTIME_ZMQ_TARGET})
endif()
policy_runtime_configure_target(policy-runtime-host)

add_executable(robot-io-health tools/robot_io_health.cpp)
policy_runtime_configure_target(robot-io-health)

add_executable(ethercat-cycle-stats tools/ethercat_cycle_stats.cpp)
target_compile_definitions(ethercat-cycle-stats PRIVATE
    POLICY_RUNTIME_WITH_IGH=$<BOOL:${POLICY_RUNTIME_WITH_IGH}>)
policy_runtime_configure_target(ethercat-cycle-stats)

install(TARGETS robot-io-daemon policy-runtime-host robot-io-health
                ethercat-cycle-stats
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(FILES configs/robot_profiles/elmo_gold_example.json
    DESTINATION "${CMAKE_INSTALL_DATADIR}/policy-runtime/robot_profiles")
install(FILES packaging/systemd/robot-io-daemon.service
    DESTINATION "${POLICY_RUNTIME_SYSTEMD_UNIT_DIR}")
install(FILES docs/robot-io-daemon.md
    DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/policy-runtime")

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

    add_executable(damiao_protocol_test tests/cpp/unit/damiao_protocol_test.cpp)
    target_link_libraries(damiao_protocol_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(damiao_protocol_test)
    add_test(NAME damiao_protocol_test COMMAND damiao_protocol_test)

    add_executable(damiao_transport_test tests/cpp/integration/damiao_transport_test.cpp)
    target_link_libraries(damiao_transport_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(damiao_transport_test)
    add_test(NAME damiao_transport_test COMMAND damiao_transport_test)

    add_executable(ipc_test tests/cpp/integration/ipc_test.cpp)
    target_link_libraries(ipc_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(ipc_test)

    add_executable(cia402_state_machine_test
        tests/cpp/unit/cia402_state_machine_test.cpp)
    target_link_libraries(cia402_state_machine_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(cia402_state_machine_test)

    add_executable(cia402_axis_test tests/cpp/unit/cia402_axis_test.cpp)
    target_link_libraries(cia402_axis_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(cia402_axis_test)

    add_executable(elmo_pdo_test tests/cpp/unit/elmo_pdo_test.cpp)
    target_link_libraries(elmo_pdo_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(elmo_pdo_test)

    add_executable(ethercat_fake_backend_test
        tests/cpp/integration/ethercat_fake_backend_test.cpp)
    target_link_libraries(ethercat_fake_backend_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(ethercat_fake_backend_test)

    add_executable(igh_sdo_semantics_test
        tests/cpp/integration/igh_sdo_semantics_test.cpp
        tests/cpp/support/igh_shim.cpp
        src/transport/ethercat/elmo_gold.cpp
        src/transport/ethercat/igh_backend.cpp
        src/transport/ethercat/master.cpp)
    target_include_directories(igh_sdo_semantics_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/cpp/support
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_definitions(igh_sdo_semantics_test PRIVATE
        POLICY_RUNTIME_WITH_IGH=1)
    target_link_libraries(igh_sdo_semantics_test PRIVATE GTest::gtest_main)
    policy_runtime_configure_target(igh_sdo_semantics_test)

    add_executable(robot_io_daemon_test
        tests/cpp/integration/robot_io_daemon_test.cpp)
    target_link_libraries(robot_io_daemon_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(robot_io_daemon_test)

    add_executable(service_config_test
        tests/cpp/integration/service_config_test.cpp)
    target_link_libraries(service_config_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    target_compile_definitions(service_config_test
        PRIVATE POLICY_RUNTIME_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
                POLICY_RUNTIME_TOOL_DIR="${CMAKE_CURRENT_BINARY_DIR}"
                POLICY_RUNTIME_DAEMON_PATH="$<TARGET_FILE:robot-io-daemon>"
                POLICY_RUNTIME_HOST_PATH="$<TARGET_FILE:policy-runtime-host>"
                POLICY_RUNTIME_FAKE_POLICY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"
                POLICY_RUNTIME_HAS_ZMQ=$<BOOL:${POLICY_RUNTIME_HAS_ZMQ}>
                POLICY_RUNTIME_SYSTEMD_UNIT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/packaging/systemd/robot-io-daemon.service")
    policy_runtime_configure_target(service_config_test)
    add_dependencies(service_config_test robot-io-daemon policy-runtime-host
                     robot-io-health ethercat-cycle-stats)

    add_executable(runtime_host_test
        tests/cpp/integration/runtime_host_test.cpp)
    target_link_libraries(runtime_host_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    target_compile_definitions(runtime_host_test
        PRIVATE POLICY_RUNTIME_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    policy_runtime_configure_target(runtime_host_test)

    add_executable(st3215_test tests/cpp/unit/st3215_test.cpp)
    target_link_libraries(st3215_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(st3215_test)

    add_executable(virtual_serial_test
        tests/cpp/integration/virtual_serial_test.cpp)
    target_link_libraries(virtual_serial_test
        PRIVATE policy_runtime_core GTest::gtest_main)
    policy_runtime_configure_target(virtual_serial_test)

    add_executable(runtime_contract_driver
        tests/cpp/integration/runtime_contract_driver.cpp)
    target_link_libraries(runtime_contract_driver PRIVATE policy_runtime_core)
    policy_runtime_configure_target(runtime_contract_driver)

    add_library(cppzmq_host_api_compile_probe OBJECT
        apps/policy_runtime_host_main.cpp)
    target_include_directories(cppzmq_host_api_compile_probe BEFORE PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/cppzmq_probe/fake-4.7")
    target_link_libraries(cppzmq_host_api_compile_probe
        PRIVATE policy_runtime_core)
    target_compile_definitions(cppzmq_host_api_compile_probe PRIVATE
        POLICY_RUNTIME_HAS_ZMQ=1)
    policy_runtime_configure_target(cppzmq_host_api_compile_probe)

    include(GoogleTest)
    gtest_discover_tests(policy_runtime_unit_tests TEST_PREFIX result_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(rpc_codec_test TEST_PREFIX rpc_codec_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(profile_loader_test TEST_PREFIX profile_loader_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(transport_scheduler_test TEST_PREFIX transport_scheduler_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(ipc_test TEST_PREFIX ipc_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(cia402_state_machine_test TEST_PREFIX cia402_state_machine_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(cia402_axis_test TEST_PREFIX cia402_axis_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(elmo_pdo_test TEST_PREFIX elmo_pdo_test.
        DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(ethercat_fake_backend_test
        TEST_PREFIX ethercat_fake_backend_test. DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(igh_sdo_semantics_test
        TEST_PREFIX igh_sdo_semantics_test. DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(robot_io_daemon_test
        TEST_PREFIX robot_io_daemon_test. DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(service_config_test
        TEST_PREFIX service_config_test. DISCOVERY_MODE PRE_TEST)
    add_test(NAME service_install_layout_test
        COMMAND "${CMAKE_COMMAND}"
            "-DPOLICY_RUNTIME_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}"
            "-DPOLICY_RUNTIME_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}"
            "-DPOLICY_RUNTIME_INSTALL_TEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/service-install-root"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/install_layout_test.cmake")
    gtest_discover_tests(runtime_host_test
        TEST_PREFIX runtime_host_test. DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(st3215_test
        TEST_PREFIX st3215_test. DISCOVERY_MODE PRE_TEST)
    gtest_discover_tests(virtual_serial_test
        TEST_PREFIX virtual_serial_test. DISCOVERY_MODE PRE_TEST)

    set(POLICY_RUNTIME_CONTRACT_PYTHON "")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin/python")
        set(POLICY_RUNTIME_CONTRACT_PYTHON
            "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin/python")
    else()
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            execute_process(
                COMMAND "${Python3_EXECUTABLE}" -c
                    "import pydantic, serial, zmq"
                RESULT_VARIABLE POLICY_RUNTIME_PYTHON_DEPS_RESULT
                OUTPUT_QUIET ERROR_QUIET)
            if(POLICY_RUNTIME_PYTHON_DEPS_RESULT EQUAL 0)
                set(POLICY_RUNTIME_CONTRACT_PYTHON "${Python3_EXECUTABLE}")
            endif()
        endif()
    endif()
    if(POLICY_RUNTIME_CONTRACT_PYTHON)
        add_test(NAME runtime_cross_language_test
            COMMAND "${POLICY_RUNTIME_CONTRACT_PYTHON}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tests/compat/run_runtime_parity.py"
                --driver $<TARGET_FILE:runtime_contract_driver>)
        set_tests_properties(runtime_cross_language_test PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" TIMEOUT 15)
        if(POLICY_RUNTIME_HAS_ZMQ)
            add_test(NAME runtime_zmq_tcp_smoke
                COMMAND "${POLICY_RUNTIME_CONTRACT_PYTHON}"
                    "${CMAKE_CURRENT_SOURCE_DIR}/tests/compat/test_runtime_zmq_smoke.py"
                    --host $<TARGET_FILE:policy-runtime-host>)
            set_tests_properties(runtime_zmq_tcp_smoke PROPERTIES
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                TIMEOUT 15 LABELS tcp_smoke)
        endif()
    endif()

    set(CPPZMQ_FIXTURE_VERSIONS 4.6 4.7)
    set(CPPZMQ_FIXTURE_EXPECTATIONS OFF ON)
    foreach(CPPZMQ_FIXTURE_MODE INCLUDE TARGET)
        foreach(CPPZMQ_FIXTURE_VERSION EXPECT_SUPPORTED IN ZIP_LISTS
                CPPZMQ_FIXTURE_VERSIONS CPPZMQ_FIXTURE_EXPECTATIONS)
            add_test(NAME cppzmq_${CPPZMQ_FIXTURE_MODE}_${CPPZMQ_FIXTURE_VERSION}_api_probe
                COMMAND "${CMAKE_COMMAND}"
                    -S "${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/cppzmq_probe"
                    -B "${CMAKE_CURRENT_BINARY_DIR}/cppzmq-probe-${CPPZMQ_FIXTURE_MODE}-${CPPZMQ_FIXTURE_VERSION}"
                    -DPOLICY_RUNTIME_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                    -DCPPZMQ_INCLUDE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/cppzmq_probe/fake-${CPPZMQ_FIXTURE_VERSION}
                    -DPROBE_MODE=${CPPZMQ_FIXTURE_MODE}
                    -DEXPECT_SUPPORTED=${EXPECT_SUPPORTED})
            set_tests_properties(
                cppzmq_${CPPZMQ_FIXTURE_MODE}_${CPPZMQ_FIXTURE_VERSION}_api_probe
                PROPERTIES LABELS build)
        endforeach()
    endforeach()
endif()
