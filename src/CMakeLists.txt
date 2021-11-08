message(STATUS "[MPUInfo]MPUDriverType select: " ${MPUDriver})

if(${MPUDriver} MATCHES "spidev")
    message(STATUS "[MPUInfo]MPUDriverType link comfirm: " ${MPUDriver})
elseif(${MPUDriver} MATCHES "pigpio")
    message(STATUS "[MPUInfo]MPUDriverType link comfirm: " ${MPUDriver})
    find_library(PIGPIO pigpio)
    add_definitions(-DMPUSPI_PIGPIO)
else()
    message(STATUS "[MPUInfo]MPUDriver no select! use spidev as default")
endif()

add_library(RPiMPU MPU9250/MPU9250.cpp)

if(${MPUDriver} MATCHES "spidev")
    # do not thing, spidev is linux build-in function
elseif(${MPUDriver} MATCHES "pigpio")
    target_link_libraries(RPiMPU ${PIGPIO})
else()
    # this is spidev
endif()