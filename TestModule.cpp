#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include "src/MPU9250/MPU9250.hpp"

int main(int argc, char const *argv[])
{
    wiringPiSetupSys();
    MPUData *myData;
    RPiMPU9250 *myMPUTest = new RPiMPU9250(); //
    myMPUTest->MPUGryoCalibration();
    myMPUTest->MPUSensorsDataGet();
    myMPUTest->ResetMPUMixAngle();

    while (true)
    {
        int microstart = micros();
        myData = myMPUTest->MPUSensorsDataGet();
        std::cout << (int)myData->_uORB_Accel__Roll << " " << (int)myData->_uORB_Accel_Pitch << " "
                  << (int)myData->_uORB_Real__Roll << " " << (int)myData->_uORB_Real_Pitch << " "
                  << (int)myData->_uORB_Gryo___Yaw << " " << (int)myData->_uORB_Gryo__Roll << " "
                  << (int)myData->_uORB_Gryo_Pitch << " " << (int)myData->_uORB_IMU_Accel_Vector << " \n";
        int microend = micros();
        usleep(4000 - (microend - microstart));
    }
}
