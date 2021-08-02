find_library(WIRING wiringPi)
find_library(PIGPIO pigpio)
aux_source_directory(MPU9250 MPU9250LIB)

add_library(RPiMPU ${MPU9250LIB})
target_link_libraries(RPiMPU ${WIRING})
target_link_libraries(RPiMPU ${PIGPIO})
