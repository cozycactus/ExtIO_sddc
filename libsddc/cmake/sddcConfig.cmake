include(FindPkgConfig)
pkg_check_modules(LIBUSB libusb-1.0 IMPORTED_TARGET)

get_filename_component(SDDC_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

if(NOT TARGET sddc::sddc)
  include("${SDDC_CMAKE_DIR}/sddcTargets.cmake")
endif()
