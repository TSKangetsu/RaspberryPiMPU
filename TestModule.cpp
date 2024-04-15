#include "Drive_Json.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include "src/MPU9250/MPU9250.hpp"
#include "src/ICM20602/ICM20602.hpp"

void configWrite(const char *configDir, const char *Target, double obj);
double configSettle(const char *configDir, const char *Target);

inline int GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

int main(int argc, char *argv[])
{
    int argvs;
    //
    int TimeStart;
    int TimeEnd;
    int TimeNext;
    int TimeMax;
    //
    ICMData myICMData;
    MPUData myData;
    //
    while ((argvs = getopt(argc, argv, "cthis")) != -1)
    {
        switch (argvs)
        {
        case 'c':
        {
            std::cout << "Start MPU Calibration\n";
            TimeMax = 500;
            MPUConfig option;
            option.MPUType = MPUTypeSPI;
            option.MPUSPIChannel = "/dev/spidev0.0";
            option.MPUI2CAddress = 0x68;
            option.MPU9250_SPI_Freq = 1000 * 1000;
            option.TargetFreqency = 1000.f;
            option.GyroToAccelBeta = 0.2;
            option.GyroDynamicAnalyse = false;
            option.GyroHardwareFilterFreq = 250;
            option.GyroFilterType = FilterLPFPT1;
            option.GyroFilterTypeST2 = FilterLPFPT1;
            option.GyroFilterCutOff = 100;
            option.GyroFilterCutOffST2 = 180;
            option.GyroFilterNotchCenterFreq = 0;
            option.GyroFilterNotchCutOff = 0;
            option.DynamicNotchQ = 0;
            option.DynamicNotchEnable = false;
            option.DynamicNotchMinFreq = 0;
            option.AccTargetFreqency = 1000.f;
            option.AccelFilterType = FilterLPFBiquad;
            option.AccelFilterCutOff = 15;
            option.MPU_Flip___Yaw = 270;
            RPiMPU9250 *myMPUTest = new RPiMPU9250(option);
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
            configWrite("./MPUCali.json", "_flag_MPU9250_A_X_Cali", tmp[MPUAccelCaliX]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_Y_Cali", tmp[MPUAccelCaliY]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_Z_Cali", tmp[MPUAccelCaliZ]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_X_Scal", tmp[MPUAccelScalX]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_Y_Scal", tmp[MPUAccelScalY]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_Z_Scal", tmp[MPUAccelScalZ]);
        }
        break;
        case 's':
        {
            TimeMax = 500;
            ICMConfig option;
            option.ICMType = ICMTypeSPI;
            option.ICMSPIChannel = "/dev/spidev0.0";
            option.ICM_Flip_Pitch = 0;
            option.ICM_Flip__Roll = 180;
            option.ICM_Flip___Yaw = 270;
            option.TargetFreqency = 1000;
            option.ICM20602_SPI_Freq = 400000;
            option.ICM_20602_LSB = 65.5 / 4;
            option.DynamicNotchEnable = true;
            option.GyroToAccelBeta = 0.2;
            option.GyroFilterNotchCutOff = 0;
            option.GyroFilterType = FilterLPFPT1;
            option.GyroFilterCutOff = 90;
            option.GyroFilterTypeST2 = FilterLPFPT1;
            option.GyroFilterCutOffST2 = 0;
            option.AccelFilterType = FilterLPFBiquad;
            option.AccTargetFreqency = 1000;
            option.AccelFilterCutOff = 30;
            option.AccelFilterNotchCutOff = 0;
            std::cout << "Start ICM Calibration\n";
            RPiICM20602 *myICMTest = new RPiICM20602(option);
            int a;
            double tmp[50];
            std::cout << "start calibration Nose Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myICMTest->ICMAccelCalibration(ICMAccelNoseUp, tmp);
            std::cout << "start calibration Nose Down and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myICMTest->ICMAccelCalibration(ICMAccelNoseDown, tmp);
            std::cout << "start calibration Nose Right Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myICMTest->ICMAccelCalibration(ICMAccelNoseRight, tmp);
            std::cout << "start calibration Nose Left Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myICMTest->ICMAccelCalibration(ICMAccelNoseLeft, tmp);
            std::cout << "start calibration Nose Top  and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myICMTest->ICMAccelCalibration(ICMAccelNoseTop, tmp);
            std::cout << "start calibration Nose Rev and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myICMTest->ICMAccelCalibration(ICMAccelNoseRev, tmp);
            myICMTest->ICMAccelCalibration(ICMAccelCaliGet, tmp);
            configWrite("./ICMCali.json", "_flag_ICM20602_A_X_Cali", tmp[ICMAccelCaliX]);
            configWrite("./ICMCali.json", "_flag_ICM20602_A_Y_Cali", tmp[ICMAccelCaliY]);
            configWrite("./ICMCali.json", "_flag_ICM20602_A_Z_Cali", tmp[ICMAccelCaliZ]);
            configWrite("./ICMCali.json", "_flag_ICM20602_A_X_Scal", tmp[ICMAccelScalX]);
            configWrite("./ICMCali.json", "_flag_ICM20602_A_Y_Scal", tmp[ICMAccelScalY]);
            configWrite("./ICMCali.json", "_flag_ICM20602_A_Z_Scal", tmp[ICMAccelScalZ]);
        }
        break;
        case 'i':
        {
            ICMConfig option;
            option.ICMType = ICMTypeSPI;
            option.ICMSPIChannel = "/dev/spidev0.0";
            option.ICM_Flip_Pitch = 0;
            option.ICM_Flip__Roll = 180;
            option.ICM_Flip___Yaw = 270;
            option.TargetFreqency = 1000;
            option.ICM20602_SPI_Freq = 400000;
            option.ICM_20602_LSB = 65.5 / 4;
            option.DynamicNotchEnable = true;
            option.GyroToAccelBeta = 0.2;
            option.GyroFilterNotchCutOff = 0;
            option.GyroFilterType = FilterLPFPT1;
            option.GyroFilterCutOff = 90;
            option.GyroFilterTypeST2 = FilterLPFPT1;
            option.GyroFilterCutOffST2 = 0;
            option.AccelFilterType = FilterLPFBiquad;
            option.AccTargetFreqency = 1000;
            option.AccelFilterCutOff = 30;
            option.AccelFilterNotchCutOff = 0;
            double AccelCaliData[30];
            RPiICM20602 *myICMTest = new RPiICM20602(option);
            AccelCaliData[ICMAccelCaliX] = configSettle("./ICMCali.json", "_flag_ICM20602_A_X_Cali");
            AccelCaliData[ICMAccelCaliY] = configSettle("./ICMCali.json", "_flag_ICM20602_A_Y_Cali");
            AccelCaliData[ICMAccelCaliZ] = configSettle("./ICMCali.json", "_flag_ICM20602_A_Z_Cali");
            AccelCaliData[ICMAccelScalX] = configSettle("./ICMCali.json", "_flag_ICM20602_A_X_Scal");
            AccelCaliData[ICMAccelScalY] = configSettle("./ICMCali.json", "_flag_ICM20602_A_Y_Scal");
            AccelCaliData[ICMAccelScalZ] = configSettle("./ICMCali.json", "_flag_ICM20602_A_Z_Scal");
            AccelCaliData[ICMAccelTRIM_Roll] = 0;
            AccelCaliData[ICMAccelTRIMPitch] = 0;
            myICMTest->ICMCalibration(AccelCaliData);
            std::cout << " Done!\n";
            myICMData = myICMTest->ICMSensorsDataGet();
            sleep(2);
            system("clear");
            
            while (true)
            {
                myICMData = myICMTest->ICMSensorsDataGet();
                usleep(5000);
                std::cout << "\033[20A";
                std::cout << "\033[K";
                std::cout << "Accel Roll: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_Accel__Roll << "|"
                          << "AccelPitch: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_Accel_Pitch << "| \n";
                std::cout << "ACC       X: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_ICM20602_A_X << "|"
                          << "ACC       Y: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_ICM20602_A_Y << "|"
                          << "ACC       Z: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_ICM20602_A_Z << "| \n";
                std::cout << "Gryo  Roll: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_Gryo__Roll << "|"
                          << "Gryo Pitch: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_Gryo_Pitch << "|"
                          << "Gryo   Yaw: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_Gryo___Yaw << "| \n";
                std::cout << "Real  Roll: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_Real__Roll << "|"
                          << "Real Pitch: " << std::setw(7) << std::setfill(' ') << (int)myICMData._uORB_Real_Pitch << "| \n"; 
            }
        }
        break;
        case 't':
        {
            double AccelCaliData[30];
            TimeMax = 1000;
            std::cout << "Start MPU Monitor\n";
            std::cout << "Setting UP MPU9250 ....";
            std::cout.flush();
            MPUConfig option;
            option.MPUType = MPUTypeSPI;
            option.MPUSPIChannel = "/dev/spidev0.0";
            option.MPUI2CAddress = 0x68;
            option.MPU9250_SPI_Freq = 1000 * 1000;
            option.TargetFreqency = 1000.f;
            option.GyroToAccelBeta = 0.2;
            option.GyroDynamicAnalyse = false;
            option.GyroHardwareFilterFreq = 250;
            option.GyroFilterType = FilterLPFPT1;
            option.GyroFilterTypeST2 = FilterLPFPT1;
            option.GyroFilterCutOff = 100;
            option.GyroFilterCutOffST2 = 180;
            option.GyroFilterNotchCenterFreq = 0;
            option.GyroFilterNotchCutOff = 0;
            option.DynamicNotchQ = 0;
            option.DynamicNotchEnable = false;
            option.DynamicNotchMinFreq = 0;
            option.AccTargetFreqency = 1000.f;
            option.AccelFilterType = FilterLPFBiquad;
            option.AccelFilterCutOff = 15;
            option.MPU_Flip___Yaw = 270;
            RPiMPU9250 *myMPUTest = new RPiMPU9250(option);
            std::cout << " Done!\n";
            //
            AccelCaliData[MPUAccelCaliX] = configSettle("./MPUCali.json", "_flag_MPU9250_A_X_Cali");
            AccelCaliData[MPUAccelCaliY] = configSettle("./MPUCali.json", "_flag_MPU9250_A_Y_Cali");
            AccelCaliData[MPUAccelCaliZ] = configSettle("./MPUCali.json", "_flag_MPU9250_A_Z_Cali");
            AccelCaliData[MPUAccelScalX] = configSettle("./MPUCali.json", "_flag_MPU9250_A_X_Scal");
            AccelCaliData[MPUAccelScalY] = configSettle("./MPUCali.json", "_flag_MPU9250_A_Y_Scal");
            AccelCaliData[MPUAccelScalZ] = configSettle("./MPUCali.json", "_flag_MPU9250_A_Z_Scal");
            AccelCaliData[MPUAccelTRIM_Roll] = 0;
            AccelCaliData[MPUAccelTRIMPitch] = 0;
            std::cout << "Calibration Gryo ......";
            std::cout.flush();
            myMPUTest->MPUCalibration(AccelCaliData);
            std::cout << " Done!\n";
            myData = myMPUTest->MPUSensorsDataGet();
            std::cout << "MPUTypeID: 0x" << std::hex << myData.DeviceType << std::dec << " \n";
            sleep(2);
            system("clear");
            while (true)
            {
                TimeStart = GetTimestamp();
                TimeNext = TimeStart - TimeEnd;
                //
                myData = myMPUTest->MPUSensorsDataGet();
                //
                std::cout << "\033[20A";
                std::cout << "\033[K";
                std::cout << "Accel Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Accel__Roll << "|"
                          << "AccelPitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Accel_Pitch << "| \n";
                std::cout << "ACC       X: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU9250_A_X << "|"
                          << "ACC       Y: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU9250_A_Y << "|"
                          << "ACC       Z: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU9250_A_Z << "| \n";
                std::cout << "Gryo  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo__Roll << "|"
                          << "Gryo Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo_Pitch << "|"
                          << "Gryo   Yaw: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo___Yaw << "| \n";
                std::cout << "Real  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real__Roll << "|"
                          << "Real Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real_Pitch << "| \n";
                //
                TimeEnd = GetTimestamp();
                if (TimeMax < ((TimeEnd - TimeStart) + TimeNext) || (TimeNext) < 0)
                    usleep(1); // In fact ,this is 50us call by linux kerenl
                else
                    usleep(TimeMax - (TimeEnd - TimeStart) - TimeNext);
                TimeEnd = GetTimestamp();
            }
        }
        break;

        case 'h':
            std::cout << "Usage: 'RaspberryPiMPU [option]'";
            std::cout << "[option] -c to calibration accel and save to MPUCali.json , -t is start to check mpu data on console\n";
            break;

        default:
            std::cout << "Usage: 'RaspberryPiMPU [option]'";
            std::cout << "[option] -c to calibration accel and save to MPUCali.json , -t is start to check mpu data on console\n";
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
