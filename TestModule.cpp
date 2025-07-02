#include "Drive_Json.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include "src/MPUProcess.hpp"

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
    MPUData myData;
    //
    while ((argvs = getopt(argc, argv, "cthsi")) != -1)
    {
        switch (argvs)
        {
        case 'c':
        {
            std::cout << "Start MPU Calibration\n";
            TimeMax = 500;
            MPUConfig option;
            option.MPUType = MPUTypeSPI;
            option.GyroScope = SensorType::AUTO;
            option.GyroSPIChannel = "/dev/spidev0.0";
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
        case 't':
        {
            double AccelCaliData[30];
            TimeMax = 1000;
            std::cout << "Start MPU Monitor\n";
            std::cout << "Setting UP ... ";
            std::cout.flush();
            MPUConfig option;
            option.GyroScope = SensorType::AUTO;
            option.GyroSPIChannel = "/dev/spidev0.0";
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
            //
            auto gyroDetected = myMPUTest->GetMPUTypeDetected();
            std::cout << "0x" << std::hex << (int)gyroDetected << " ... Done!\n";
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
            
            // Check if debug mode is requested
            bool debug_mode = false;
            for (int i = 1; i < argc; i++) {
                if (std::string(argv[i]) == "-d" || std::string(argv[i]) == "--debug") {
                    debug_mode = true;
                    break;
                }
            }
            if (debug_mode) {
                myMPUTest->SetDebugOutput(true);
                std::cout << "Debug mode enabled - detailed output will be shown\n";
            }
            
            // Check calibration and offer recalibration if needed
            if (!myMPUTest->IsAccelerometerCalibrated()) {
                std::cout << "WARNING: Accelerometer calibration appears to be incorrect!\n";
                std::cout << "Raw Z value should be ~1.0g when stationary, but is showing 0.429g\n";
                
                // Show current calibration values
                std::cout << "Current calibration values:\n";
                std::cout << "  X Scale: " << AccelCaliData[MPUAccelScalX] << "\n";
                std::cout << "  Y Scale: " << AccelCaliData[MPUAccelScalY] << "\n";
                std::cout << "  Z Scale: " << AccelCaliData[MPUAccelScalZ] << "\n";
                std::cout << "  X Offset: " << AccelCaliData[MPUAccelCaliX] << "\n";
                std::cout << "  Y Offset: " << AccelCaliData[MPUAccelCaliY] << "\n";
                std::cout << "  Z Offset: " << AccelCaliData[MPUAccelCaliZ] << "\n";
                
                std::cout << "Would you like to recalibrate? (y/n): ";
                char response;
                std::cin >> response;
                if (response == 'y' || response == 'Y') {
                    std::cout << "Please run calibration mode first: ./RaspberryPiMPU -c\n";
                    std::cout << "Then restart this test mode.\n";
                    delete myMPUTest;
                    return 0;
                }
            }
            
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
                std::cout << "ACC       X: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU9250_AC_X << "|"
                          << "ACC       Y: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU9250_AC_Y << "|"
                          << "ACC       Z: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU9250_AC_Z << "| \n";
                std::cout << "Gryo  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo__Roll << "|"
                          << "Gryo Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo_Pitch << "|"
                          << "Gryo   Yaw: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo___Yaw << "| \n";
                std::cout << "Real  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real__Roll << "|"
                          << "Real Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real_Pitch << "| \n";
                
                // Earth frame acceleration display
                std::cout << "Earth Accel X: " << std::setw(7) << std::setfill(' ') << std::fixed << std::setprecision(1) << myData._uORB_Acceleration_X << "|"
                          << "Earth Accel Y: " << std::setw(7) << std::setfill(' ') << std::fixed << std::setprecision(1) << myData._uORB_Acceleration_Y << "|"
                          << "Earth Accel Z: " << std::setw(7) << std::setfill(' ') << std::fixed << std::setprecision(1) << myData._uORB_Acceleration_Z << "| \n";
                
                // Get pure acceleration (without gravity)
                float ax_pure, ay_pure, az_pure;
                myMPUTest->GetPureAcceleration(ax_pure, ay_pure, az_pure);
                std::cout << "Pure Accel X: " << std::setw(7) << std::setfill(' ') << std::fixed << std::setprecision(1) << ax_pure << "|"
                          << "Pure Accel Y: " << std::setw(7) << std::setfill(' ') << std::fixed << std::setprecision(1) << ay_pure << "|"
                          << "Pure Accel Z: " << std::setw(7) << std::setfill(' ') << std::fixed << std::setprecision(1) << az_pure << "| \n";
                
                // Debug information (optional)
                if (myMPUTest->IsAccelerometerCalibrated()) {
                    std::cout << "Calibration: OK | ";
                } else {
                    std::cout << "Calibration: FAIL | ";
                }
                
                // Show raw accelerometer values
                float ax_raw, ay_raw, az_raw;
                myMPUTest->GetRawAccelerometerValues(ax_raw, ay_raw, az_raw);
                std::cout << "Raw (g): X=" << std::setw(6) << std::setfill(' ') << std::fixed << std::setprecision(3) << ax_raw 
                         << " Y=" << std::setw(6) << std::setfill(' ') << std::fixed << std::setprecision(3) << ay_raw 
                         << " Z=" << std::setw(6) << std::setfill(' ') << std::fixed << std::setprecision(3) << az_raw << "| \n";
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
            std::cout << "Additional options for -t mode:\n";
            std::cout << "  -d or --debug: Enable detailed debug output\n";
            std::cout << "Example: ./RaspberryPiMPU -t -d\n";
            break;

        default:
            std::cout << "Usage: 'RaspberryPiMPU [option]'";
            std::cout << "[option] -c to calibration accel and save to MPUCali.json , -t is start to check mpu data on console\n";
            std::cout << "Additional options for -t mode:\n";
            std::cout << "  -d or --debug: Enable detailed debug output\n";
            std::cout << "Example: ./RaspberryPiMPU -t -d\n";
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
