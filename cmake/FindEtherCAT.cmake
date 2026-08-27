include(FindPackageHandleStandardArgs)

find_path(EtherCAT_INCLUDE_DIR
    NAMES ecrt.h
    HINTS ${EtherCAT_ROOT} ENV EtherCAT_ROOT ENV ETHERCAT_ROOT
    PATH_SUFFIXES include include/ethercat)

find_library(EtherCAT_LIBRARY
    NAMES ethercat
    HINTS ${EtherCAT_ROOT} ENV EtherCAT_ROOT ENV ETHERCAT_ROOT
    PATH_SUFFIXES lib lib64)

find_package_handle_standard_args(EtherCAT
    REQUIRED_VARS EtherCAT_INCLUDE_DIR EtherCAT_LIBRARY
    REASON_FAILURE_MESSAGE
        "POLICY_RUNTIME_WITH_IGH=ON requires the IgH EtherCAT userspace development header ecrt.h and libethercat. Install them or set EtherCAT_ROOT")

if(EtherCAT_FOUND AND NOT TARGET EtherCAT::EtherCAT)
    add_library(EtherCAT::EtherCAT UNKNOWN IMPORTED)
    set_target_properties(EtherCAT::EtherCAT PROPERTIES
        IMPORTED_LOCATION "${EtherCAT_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${EtherCAT_INCLUDE_DIR}")
endif()

mark_as_advanced(EtherCAT_INCLUDE_DIR EtherCAT_LIBRARY)
