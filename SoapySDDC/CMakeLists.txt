# SoapySDR Lib

find_package(SoapySDR CONFIG)

if (SoapySDR_FOUND)

    include_directories("." "../Core")

    SOAPY_SDR_MODULE_UTIL(
        TARGET SDDCSupport
        SOURCES
        SoapySDDC.hpp
        Registration.cpp
        Settings.cpp
        Streaming.cpp
        LIBRARIES
        SDDC_CORE
    )

    add_dependencies(SDDCSupport SDDC_CORE)

    target_link_libraries(SDDCSupport PRIVATE SDDC_CORE)
    set_target_properties(SDDCSupport PROPERTIES VERSION ${PROJECT_VERSION})
    set_target_properties(SDDCSupport PROPERTIES SOVERSION 1)

endif(SoapySDR_FOUND)