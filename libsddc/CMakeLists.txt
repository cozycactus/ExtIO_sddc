cmake_minimum_required(VERSION 3.13)

include(GNUInstallDirs)
include(GenerateExportHeader)
include(CMakePackageConfigHelpers)

find_package(Threads)
find_package(PkgConfig)

if (MSVC)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif (MSVC)

include_directories("." "../Core")

add_library(sddc SHARED
    libsddc.cpp
)

target_link_libraries(sddc PRIVATE SDDC_CORE)
set_target_properties(sddc PROPERTIES VERSION ${PROJECT_VERSION} PUBLIC_HEADER "libsddc.h")
set_target_properties(sddc PROPERTIES SOVERSION 0)

add_library(sddc_static STATIC
    libsddc.cpp
    ${CMAKE_SOURCE_DIR}/SDDC_FX3/SDDC_FX3.h
    $<TARGET_OBJECTS:SDDC_CORE>
)
set_property(TARGET sddc_static PROPERTY POSITION_INDEPENDENT_CODE ON)
add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/SDDC_FX3/SDDC_FX3.h COMMAND make WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/SDDC_FX3)
set_property(TARGET sddc_static PROPERTY ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_SOURCE_DIR}/SDDC_FX3/SDDC_FX3.h)
set_property(TARGET sddc_static  PROPERTY OUTPUT_NAME sddc)
set_target_properties(sddc_static PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(sddc_static PROPERTIES SOVERSION 0)

# Install libraries and export targets
install(TARGETS sddc sddc_static
    EXPORT SDDC-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
    INCLUDES DESTINATION include
)




# Export targets
install(EXPORT SDDC-export
    FILE sddcTargets.cmake
    NAMESPACE sddc::
    DESTINATION lib/cmake/sddc
)

########################################################################
# Create Pkg Config File
########################################################################
FOREACH(inc ${LIBUSB_INCLUDEDIR})
    LIST(APPEND SDDC_PC_CFLAGS "-I${inc}")
ENDFOREACH(inc)

FOREACH(lib ${LIBUSB_LIBRARY_DIRS})
    LIST(APPEND SDDC_PC_LIBS "-L${lib}")
ENDFOREACH(lib)

# use space-separation format for the pc file
STRING(REPLACE ";" " " SDDC_PC_CFLAGS "${SDDC_PC_CFLAGS}")
STRING(REPLACE ";" " " SDDC_PC_LIBS "${SDDC_PC_LIBS}")


# Set variables for pkg-config
set(prefix "${CMAKE_INSTALL_PREFIX}")
set(exec_prefix \${prefix})
set(libdir \${exec_prefix}/lib)
set(includedir \${prefix}/include)

# Configure the pkg-config file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libsddc.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libsddc.pc
    @ONLY
)

# Install the pkg-config file
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/libsddc.pc
    DESTINATION lib/pkgconfig
)

########################################################################
# Create CMake Config File
########################################################################
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/sddc/sddcConfigVersion.cmake"
  VERSION ${VERSION}
  COMPATIBILITY AnyNewerVersion
  )

configure_file(cmake/sddcConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/sddc/sddcConfig.cmake"
  COPYONLY
  )

set(ConfigPackageLocation lib/cmake/sddc)
install(EXPORT SDDC-export
  FILE sddcTargets.cmake
  NAMESPACE sddc::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sddc/
  )
install(
  FILES
  cmake/sddcConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/sddc/sddcConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sddc/
  COMPONENT Devel
  )

########################################################################
# Print Summary
########################################################################
MESSAGE(STATUS "Building for version: ${VERSION} / ${LIBVER}")
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")


add_library(wavewriter STATIC wavewrite.c)

# applications
add_executable(sddc_test sddc_test.c)
target_link_libraries(sddc_test PRIVATE sddc ${ASANLIB})

add_executable(sddc_stream_test sddc_stream_test.c)
target_link_libraries(sddc_stream_test sddc ${ASANLIB} wavewriter)

add_executable(sddc_vhf_stream_test sddc_vhf_stream_test.c)
target_link_libraries(sddc_vhf_stream_test sddc ${ASANLIB} wavewriter)
