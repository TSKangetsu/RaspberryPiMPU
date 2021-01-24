#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <wiringPi.h>
#include "src/MPU9250/MPU9250.hpp"

void configWrite(const char *configDir, const char *Target, double obj);
double configSettle(const char *configDir, const char *Target);

int main(int argc, char *argv[])
{
    wiringPiSetupSys();
    int argvs;
    //
    int TimeStart;
    int TimeEnd;
    int TimeNext;
    int TimeMax;
    //
    MPUData myData;
    //
    while ((argvs = getopt(argc, argv, "c:t:")) != -1)
    {
        switch (argvs)
        {
        case 'c':
        {
            std::cout << "Start MPU Calibration\n";
            TimeMax = std::atoi(optarg);
            RPiMPU9250 *myMPUTest = new RPiMPU9250(1, false, 1, 0x68, TimeMax, 0);
            int a;
            double tmp[50];
            std::cout << "start calibration Nose Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseUp, tmp);
            std::cout << "start calibration Nose Down and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseDown, tmp);
            std::cout << "start calibration Nose Right Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseRight, tmp);
            std::cout << "start calibration Nose Left Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseLeft, tmp);
            std::cout << "start calibration Nose Top  and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseTop, tmp);
            std::cout << "start calibration Nose Rev and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseRev, tmp);
            myMPUTest->MPUAccelCalibration(MPUAccelCaliGet, tmp);
            for (size_t i = 0; i < 15; i++)
            {
                std::cout << tmp[i] << " \n";
            }
            configWrite("../MPUCali.json", "_flag_MPU9250_A_X_Cali", tmp[MPUAccelCaliX]);
            configWrite("../MPUCali.json", "_flag_MPU9250_A_Y_Cali", tmp[MPUAccelCaliY]);
            configWrite("../MPUCali.json", "_flag_MPU9250_A_Z_Cali", tmp[MPUAccelCaliZ]);
            configWrite("../MPUCali.json", "_flag_MPU9250_A_X_Scal", tmp[MPUAccelScalX]);
            configWrite("../MPUCali.json", "_flag_MPU9250_A_Y_Scal", tmp[MPUAccelScalY]);
            configWrite("../MPUCali.json", "_flag_MPU9250_A_Z_Scal", tmp[MPUAccelScalZ]);
        }
        break;
        case 't':
        {
            double AccelCaliData[30];
            TimeMax = std::atoi(optarg);
            std::cout << "Start MPU Monitor\n";
            std::cout << "Setting UP MPU9250 ....";
            std::cout.flush();
            RPiMPU9250 *myMPUTest = new RPiMPU9250(1, false, 1, 0x68, TimeMax, 0);
            std::cout << " Done!\n";
            //
            AccelCaliData[MPUAccelCaliX] = configSettle("../MPUCali.json", "_flag_MPU9250_A_X_Cali");
            AccelCaliData[MPUAccelCaliY] = configSettle("../MPUCali.json", "_flag_MPU9250_A_Y_Cali");
            AccelCaliData[MPUAccelCaliZ] = configSettle("../MPUCali.json", "_flag_MPU9250_A_Z_Cali");
            AccelCaliData[MPUAccelScalX] = configSettle("../MPUCali.json", "_flag_MPU9250_A_X_Scal");
            AccelCaliData[MPUAccelScalY] = configSettle("../MPUCali.json", "_flag_MPU9250_A_Y_Scal");
            AccelCaliData[MPUAccelScalZ] = configSettle("../MPUCali.json", "_flag_MPU9250_A_Z_Scal");
            std::cout << "Calibration Gryo ......";
            std::cout.flush();
            myMPUTest->MPUCalibration(AccelCaliData);
            std::cout << " Done!\n";
            sleep(1);
            //
            myMPUTest->MPUSensorsDataGet();
            myMPUTest->ResetMPUMixAngle();
            //
            system("clear");
            while (true)
            {
                TimeStart = micros();
                TimeNext = TimeStart - TimeEnd;
                //
                myData = myMPUTest->MPUSensorsDataGet();
                //
                std::cout << "\033[20A";
                std::cout << "\033[K";
                std::cout << "Accel Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Accel__Roll << "|"
                          << "AccelPitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Accel_Pitch << "| \n";
                std::cout << "Gryo  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo__Roll << "|"
                          << "Gryo Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo_Pitch << "|"
                          << "Gryo   Yaw: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo___Yaw << "| \n";
                std::cout << "Real  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real__Roll << "|"
                          << "Real Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real_Pitch << "| \n";
                //
                TimeEnd = micros();
                if (TimeMax < ((TimeEnd - TimeStart) + TimeNext) || (TimeNext) < 0)
                    usleep(50);
                else
                    usleep(TimeMax - (TimeEnd - TimeStart) - TimeNext);
                TimeEnd = micros();
            }
        }
        break;
        }
    }
}

double configSettle(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<double>();
}

void configWrite(const char *configDir, const char *Target, double obj)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    Configdata[Target] = obj;
    std::ofstream configs;
    configs.open(configDir);
    configs << std::setw(4) << Configdata << std::endl;
    configs.close();
}
