include(CheckCXXSourceCompiles)

function(policy_runtime_check_cppzmq_api result_variable)
    set(one_value_arguments TARGET INCLUDE_DIR)
    cmake_parse_arguments(
        CPPZMQ_PROBE "" "${one_value_arguments}" "" ${ARGN})

    if(CPPZMQ_PROBE_TARGET AND CPPZMQ_PROBE_INCLUDE_DIR)
        message(FATAL_ERROR
            "cppzmq API probe accepts either TARGET or INCLUDE_DIR, not both")
    endif()
    if(NOT CPPZMQ_PROBE_TARGET AND NOT CPPZMQ_PROBE_INCLUDE_DIR)
        message(FATAL_ERROR
            "cppzmq API probe requires TARGET or INCLUDE_DIR")
    endif()

    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
    set(CMAKE_REQUIRED_QUIET TRUE)
    if(CPPZMQ_PROBE_TARGET)
        set(CMAKE_REQUIRED_LIBRARIES "${CPPZMQ_PROBE_TARGET}")
    else()
        set(CMAKE_REQUIRED_INCLUDES "${CPPZMQ_PROBE_INCLUDE_DIR}")
    endif()

    unset(${result_variable} CACHE)
    check_cxx_source_compiles([=[
        #include <string>
        #include <string_view>
        #include <zmq.hpp>

        int main() {
          zmq::context_t context{1};
          zmq::socket_t socket{context, zmq::socket_type::rep};
          socket.set(zmq::sockopt::linger, 0);
          socket.set(zmq::sockopt::rcvtimeo, 100);
          socket.set(zmq::sockopt::sndtimeo, 100);
          socket.bind("tcp://127.0.0.1:5555");

          zmq::message_t request;
          auto received = socket.recv(request, zmq::recv_flags::none);
          const std::string_view request_text(
              static_cast<const char*>(request.data()), request.size());
          std::string response(request_text);
          auto sent = socket.send(zmq::buffer(response), zmq::send_flags::none);
          try {
            return received.has_value() && sent.has_value() ? 0 : 1;
          } catch (const zmq::error_t& error) {
            return error.num();
          }
        }
    ]=] "${result_variable}")

    set(${result_variable} "${${result_variable}}" PARENT_SCOPE)
endfunction()
