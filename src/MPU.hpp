#pragma once
#include "_thirdparty/libeigen/Eigen/Dense"
#include "_thirdparty/libeigen/Eigen/LU"

#define MPU9250_ACCEL_LSB 2048.f
#define MPU9250_GYRO_LSB 16.4

#define MPUTypeI2C 0
#define MPUTypeSPI 1
#define MPUMixKalman 1
#define MPUMixTradition 0
#define MPUAccelNoseUp 0
#define MPUAccelNoseDown 1
#define MPUAccelNoseRight 2
#define MPUAccelNoseLeft 3
#define MPUAccelNoseTop 4
#define MPUAccelNoseRev 5
#define MPUAccelCaliGet 6
#define MPUAccelCaliX 7
#define MPUAccelCaliY 8
#define MPUAccelCaliZ 9
#define MPUAccelScalX 10
#define MPUAccelScalY 11
#define MPUAccelScalZ 12
#define MPUAccelTRIM_Roll 13
#define MPUAccelTRIMPitch 14

#define FilterLPFPT1 0
#define FilterLPFBiquad 1

#define FFTResolution 31.25
#define FFTBINCOUNT 16
#define GravityAccel 9.80665
#define MPU_250HZ_LPF_SPEED 8000.f
#define MPU_LOWHZ_LPF_SPEED 1000.f

#define DYNAMIC_NOTCH_DEFAULT_CENTER_HZ 350.f
#define DYN_NOTCH_SMOOTH_FREQ_HZ 50.f
#define ACC_VIBE_FLOOR_FILT_HZ 5.f
#define ACC_VIBE_FILT_HZ 2.f
#define MAX_ACC_NEARNESS 0.33 // 33% or G error soft-accepted (0.67-1.33G)
#define ACC_CLIPPING_THRESHOLD_G 7.9f

#ifndef M_Ef
#define M_Ef 2.71828182845904523536f
#endif

#define GAXR 0
#define GAYP 1
#define GAZY 2
#define AAXN 0
#define AAYN 1
#define AAZN 2

#define DEG2RAD(x) (x * PI / 180.f)

#define MPU_250HZ_LPF_SPEED 8000.f
#define MPUTypeI2C 0
#define MPUTypeSPI 1

enum SensorType
{
    AUTO,
    MPU9250 = 0x70,
    MPU9255 = 0x71,
    MPU6500 = 0x68,
    ICM20602 = 0x12,
    ICM42605 = 0x42,
};

struct MPUConfig
{
    int MPUType = MPUTypeSPI;
    const char *GyroSPIChannel = "/dev/spidev0.0";
    uint8_t MPUI2CAddress = 0x68;
    int MPU9250_SPI_Freq = 400000;
    float MPU_Flip_Pitch = 0;
    float MPU_Flip__Roll = 0;
    float MPU_Flip___Yaw = 0;
    //
    int TargetFreqency = 1000;
    float GyroToAccelBeta = 0.02;
    float MagToYawBeta = 0.02;
    bool GyroDynamicAnalyse = false;
    //
    int GyroHardwareFilterFreq = 250;
    //
    int GyroFilterType = FilterLPFPT1;
    int GyroFilterCutOff = 90;
    int GyroFilterTypeST2 = FilterLPFPT1;
    int GyroFilterCutOffST2 = 0;
    int GyroFilterNotchCenterFreq = 150;
    int GyroFilterNotchCutOff = 0;
    //
    float DynamicNotchQ = 1.2;
    float DynamicNotchMinFreq = 62.5;
    bool DynamicNotchEnable = true;
    //
    int AccTargetFreqency = 1000;
    int AccelFilterType = FilterLPFBiquad;
    int AccelFilterCutOff = 30;
    int AccelFilterNotchCenterFreq = 0;
    int AccelFilterNotchCutOff = 0;
    SensorType GyroScope = SensorType::MPU9250;
};

struct MPUData
{
    int DeviceType = 0;

    int _uORB_MPU9250_A_X = 0;
    int _uORB_MPU9250_A_Y = 0;
    int _uORB_MPU9250_A_Z = 0;
    int _uORB_MPU9250_G_X = 0;
    int _uORB_MPU9250_G_Y = 0;
    int _uORB_MPU9250_G_Z = 0;
    int _uORB_MPU9250_M_X = 0;
    int _uORB_MPU9250_M_Y = 0;
    int _uORB_MPU9250_M_Z = 0;

    float _uORB_MPU9250_ADF_X = 0;
    float _uORB_MPU9250_ADF_Y = 0;
    float _uORB_MPU9250_ADF_Z = 0;
    bool _uORB_MPU9250_ACC_Clipped = false;

    float _uORB_MPU9250_A_Static_X = 0;
    float _uORB_MPU9250_A_Static_Y = 0;
    float _uORB_MPU9250_A_Static_Z = 0;
    float _uORB_MPU9250_A_Vector = 0;
    float _uORB_MPU9250_A_Static_Vector = 0;

    float _uORB_Gryo__Roll = 0;
    float _uORB_Gryo_Pitch = 0;
    float _uORB_Gryo___Yaw = 0;
    float _uORB_Real__Roll = 0;
    float _uORB_Real_Pitch = 0;
    float _uORB_Real___Yaw = 0;
    float _uORB_Accel__Roll = 0;
    float _uORB_Accel_Pitch = 0;
    float _uORB_Acceleration_X = 0;
    float _uORB_Acceleration_Y = 0;
    float _uORB_Acceleration_Z = 0;
    float _uORB_Raw_QuaternionQ[4] = {0};
    float MPUMixTraditionBeta = 0.05;

    float _uORB_Accel_VIBE_X = 0;
    float _uORB_Accel_VIBE_Y = 0;
    float _uORB_Accel_VIBE_Z = 0;

    int _flag_MPU9250_G_X_Cali;
    int _flag_MPU9250_G_Y_Cali;
    int _flag_MPU9250_G_Z_Cali;

    double _flag_MPU9250_A_X_Scal = 1.f;
    double _flag_MPU9250_A_Y_Scal = 1.f;
    double _flag_MPU9250_A_Z_Scal = 1.f;
    double _flag_MPU9250_A_X_Cali = 0;
    double _flag_MPU9250_A_Y_Cali = 0;
    double _flag_MPU9250_A_Z_Cali = 0;
    double _flag_MPU9250_A_TP_Cali = 0;
    double _flag_MPU9250_A_TR_Cali = 0;

    // float FFTSampleBox[3][25] = {{0}};
    int fftindexs[3] = {0};
    std::vector<std::vector<float>> FFTSampleBox;
    float _uORB_Gyro_Dynamic_NotchCenterHZ[3] = {350, 350, 350};
    Eigen::Matrix3d _uORB_MPU9250_RotationMatrix;
    Eigen::Quaternion<double> _uORB_MPU9250_Quaternion;

    int _uORB_MPU9250_IMUUpdateTime = 0;
    int _uORB_MPU9250_AccelCountDown = 0;
    int _uORB_MPU9250_CalibrationCountDown = 0;
};
