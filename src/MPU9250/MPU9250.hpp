#include <math.h>
#include <vector>
#include <thread>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>

#include "filter.h"
#include "FFTPlugin.hpp"
#include "MadgwickAHRS.hpp"
#include "../_thirdparty/libeigen/Eigen/Dense"
#include "../_thirdparty/libeigen/Eigen/LU"

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

#define FilterLPFPT1 0
#define FilterLPFBiquad 1

#define GravityAccel 9.80665
#define MPU_250HZ_LPF_SPEED 8000.f
#define MPU_LOWHZ_LPF_SPEED 1000.f

#define DYNAMIC_NOTCH_DEFAULT_CENTER_HZ 350.f
#define DYN_NOTCH_SMOOTH_FREQ_HZ 30.f
#define ACC_VIBE_FLOOR_FILT_HZ 5.f
#define ACC_VIBE_FILT_HZ 2.f
#define IMU_CALI_MAX_LOOP 500.0
#define MAX_ACC_NEARNESS 0.33 // 33% or G error soft-accepted (0.67-1.33G)
#define M_Ef 2.71828182845904523536f

struct MPUConfig
{
    int MPUType = MPUTypeSPI;
    int MPUSPIChannel = 1;
    uint8_t MPUI2CAddress = 0x68;
    //
    int TargetFreqency = 1000;
    float GyroToAccelBeta = 0.02;
    bool GyroDynamicAnalyse = false;
    //
    int GyroHardwareFilterFreq = 250;
    //
    int GyroFilterType = FilterLPFPT1;
    int GyroFilterCutOff = 90;
    int GyroFilterNotchCenterFreq = 150;
    int GyroFilterNotchCutOff = 0;
    //
    float DynamicNotchQ = 1.2;
    bool DynamicNotchEnable = true;
    int DynamicNotchMinFreq = 60;
    //
    int AccelFilterType = FilterLPFBiquad;
    int AccelFilterCutOff = 30;
};

struct MPUData
{
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

    float FFTSampleBox[3][25] = {{0}};
    float _uORB_Gyro_Dynamic_NotchCenterHZ[3] = {350};
    Eigen::Matrix3d _uORB_MPU9250_RotationMatrix;
    Eigen::Quaternion<double> _uORB_MPU9250_Quaternion;
};

class RPiMPU9250
{
public:
    inline RPiMPU9250(MPUConfig mpuConfig)
    {
        //copy args
        {
            PrivateConfig = mpuConfig;
        }
        //Settings all Filter
        {
            int DT = (float)(1.f / (float)PrivateConfig.TargetFreqency) * 1000000;
            switch (PrivateConfig.GyroFilterType)
            {
            case FilterLPFPT1:
                pt1FilterInit(&GryoFilterLPFX, PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPFY, PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPFZ, PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                break;

            case FilterLPFBiquad:
                biquadFilterInitLPF(&GryoFilterBLPFX, PrivateConfig.GyroFilterCutOff, DT);
                biquadFilterInitLPF(&GryoFilterBLPFY, PrivateConfig.GyroFilterCutOff, DT);
                biquadFilterInitLPF(&GryoFilterBLPFZ, PrivateConfig.GyroFilterCutOff, DT);
                break;
            }
            switch (PrivateConfig.AccelFilterType)
            {
            case FilterLPFPT1:
                pt1FilterInit(&AccelFilterLPFX, PrivateConfig.AccelFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&AccelFilterLPFY, PrivateConfig.AccelFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&AccelFilterLPFZ, PrivateConfig.AccelFilterCutOff, DT * 1e-6f);
                break;

            case FilterLPFBiquad:
                biquadFilterInitLPF(&AccelFilterBLPFX, PrivateConfig.AccelFilterCutOff, DT);
                biquadFilterInitLPF(&AccelFilterBLPFY, PrivateConfig.AccelFilterCutOff, DT);
                biquadFilterInitLPF(&AccelFilterBLPFZ, PrivateConfig.AccelFilterCutOff, DT);
                break;
            }

            if (PrivateConfig.GyroFilterNotchCutOff)
            {
                biquadFilterInitNotch(&GyroNotchPitch, DT, PrivateConfig.GyroFilterNotchCenterFreq, PrivateConfig.GyroFilterNotchCutOff);
                biquadFilterInitNotch(&GyroNotch_Roll, DT, PrivateConfig.GyroFilterNotchCenterFreq, PrivateConfig.GyroFilterNotchCutOff);
                biquadFilterInitNotch(&GyroNotch__Yaw, DT, PrivateConfig.GyroFilterNotchCenterFreq, PrivateConfig.GyroFilterNotchCutOff);
            }
            if (PrivateConfig.DynamicNotchEnable)
            {
                biquadFilterInitLPF(&GryoFilterDynamicFreq[0], DYN_NOTCH_SMOOTH_FREQ_HZ, ((float)DT * ((float)(PrivateConfig.TargetFreqency / 1000) * 2.f)));
                biquadFilterInitLPF(&GryoFilterDynamicFreq[1], DYN_NOTCH_SMOOTH_FREQ_HZ, ((float)DT * ((float)(PrivateConfig.TargetFreqency / 1000) * 2.f)));
                biquadFilterInitLPF(&GryoFilterDynamicFreq[2], DYN_NOTCH_SMOOTH_FREQ_HZ, ((float)DT * ((float)(PrivateConfig.TargetFreqency / 1000) * 2.f)));
                biquadFilterInit(&GryoFilterDynamicNotchX, DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
                biquadFilterInit(&GryoFilterDynamicNotchY, DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
                biquadFilterInit(&GryoFilterDynamicNotchZ, DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
            }
            pt1FilterInit(&VibeFloorLPFX, ACC_VIBE_FLOOR_FILT_HZ, DT * 1e-6f);
            pt1FilterInit(&VibeFloorLPFY, ACC_VIBE_FLOOR_FILT_HZ, DT * 1e-6f);
            pt1FilterInit(&VibeFloorLPFZ, ACC_VIBE_FLOOR_FILT_HZ, DT * 1e-6f);
            pt1FilterInit(&VibeLPFX, ACC_VIBE_FILT_HZ, DT * 1e-6f);
            pt1FilterInit(&VibeLPFY, ACC_VIBE_FILT_HZ, DT * 1e-6f);
            pt1FilterInit(&VibeLPFZ, ACC_VIBE_FILT_HZ, DT * 1e-6f);
        }
        // MPUInit
        IMUSensorsDeviceInit();
        AHRSSys = new MadgwickAHRS(PrivateConfig.GyroToAccelBeta, PrivateConfig.TargetFreqency);
    };

    // Gryo must be Calibration Before Get MPU Data, This Function Require a Correctly Accel Calibration
    // See TestModule.cpp
    inline int MPUCalibration(double *AccelCaliData)
    {
        PrivateData._flag_MPU9250_A_X_Scal = AccelCaliData[MPUAccelScalX];
        PrivateData._flag_MPU9250_A_Y_Scal = AccelCaliData[MPUAccelScalY];
        PrivateData._flag_MPU9250_A_Z_Scal = AccelCaliData[MPUAccelScalZ];
        PrivateData._flag_MPU9250_A_X_Cali = AccelCaliData[MPUAccelCaliX];
        PrivateData._flag_MPU9250_A_Y_Cali = AccelCaliData[MPUAccelCaliY];
        PrivateData._flag_MPU9250_A_Z_Cali = AccelCaliData[MPUAccelCaliZ];

        double _Tmp_Gryo_X_Cali = 0;
        double _Tmp_Gryo_Y_Cali = 0;
        double _Tmp_Gryo_Z_Cali = 0;
        double _Tmp_Accel_Static_Cali = 0;
        PrivateData._flag_MPU9250_G_X_Cali = 0;
        PrivateData._flag_MPU9250_G_Y_Cali = 0;
        PrivateData._flag_MPU9250_G_Z_Cali = 0;

        for (int cali_count = 0; cali_count < IMU_CALI_MAX_LOOP; cali_count++)
        {
            MPUSensorsDataGet();
            ResetMPUMixAngle();
            _Tmp_Gryo_X_Cali += PrivateData._uORB_MPU9250_G_X;
            _Tmp_Gryo_Y_Cali += PrivateData._uORB_MPU9250_G_Y;
            _Tmp_Gryo_Z_Cali += PrivateData._uORB_MPU9250_G_Z;
            _Tmp_Accel_Static_Cali += PrivateData._uORB_MPU9250_A_Vector;
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }
        PrivateData._flag_MPU9250_G_X_Cali = _Tmp_Gryo_X_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._flag_MPU9250_G_Y_Cali = _Tmp_Gryo_Y_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._flag_MPU9250_G_Z_Cali = _Tmp_Gryo_Z_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._uORB_MPU9250_A_Static_Vector = _Tmp_Accel_Static_Cali / IMU_CALI_MAX_LOOP;
        return 0;
    };

    // Calibration MPU Accel Sensor , See TestMoodule.cpp
    inline void MPUAccelCalibration(int AccelCaliAction, double *AccelCaliData)
    {
        int AccelCaliTmpTotal = 0;
        AccelCaliData[AccelCaliAction] = 0;
        for (int cali_count = 0; cali_count < 2000; cali_count++)
        {
            IMUSensorsDataRead();
            switch (AccelCaliAction)
            {
            case MPUAccelNoseUp:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_Y;
                break;
            case MPUAccelNoseDown:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_Y;
                break;
            case MPUAccelNoseRight:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_X;
                break;
            case MPUAccelNoseLeft:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_X;
                break;
            case MPUAccelNoseTop:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_Z;
                break;
            case MPUAccelNoseRev:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_Z;
                break;
            }
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }
        if (AccelCaliAction == MPUAccelCaliGet)
        {
            AccelCaliData[MPUAccelNoseUp] /= 2000.f;
            AccelCaliData[MPUAccelNoseDown] /= 2000.f;
            AccelCaliData[MPUAccelNoseRight] /= 2000.f;
            AccelCaliData[MPUAccelNoseLeft] /= 2000.f;
            AccelCaliData[MPUAccelNoseTop] /= 2000.f;
            AccelCaliData[MPUAccelNoseRev] /= 2000.f;
            AccelCaliData[MPUAccelCaliX] = (AccelCaliData[MPUAccelNoseRight] + AccelCaliData[MPUAccelNoseLeft]) / 2.f;
            AccelCaliData[MPUAccelCaliY] = (AccelCaliData[MPUAccelNoseUp] + AccelCaliData[MPUAccelNoseDown]) / 2.f;
            AccelCaliData[MPUAccelCaliZ] = (AccelCaliData[MPUAccelNoseTop] + AccelCaliData[MPUAccelNoseRev]) / 2.f;
            AccelCaliData[MPUAccelScalX] = MPU9250_Accel_LSB / (AccelCaliData[MPUAccelNoseLeft] - AccelCaliData[7]);
            AccelCaliData[MPUAccelScalY] = MPU9250_Accel_LSB / (AccelCaliData[MPUAccelNoseUp] - AccelCaliData[8]);
            AccelCaliData[MPUAccelScalZ] = MPU9250_Accel_LSB / (AccelCaliData[MPUAccelNoseTop] - AccelCaliData[9]);
        }
    }

    // Get MPU Data, If you want a Vibration Insensitive data , you need a Stable Loop
    // See TestModule.cpp
    inline MPUData MPUSensorsDataGet()
    {
        IMUSensorsDataRead();
        PrivateData._uORB_MPU9250_G_X -= PrivateData._flag_MPU9250_G_X_Cali;
        PrivateData._uORB_MPU9250_G_Y -= PrivateData._flag_MPU9250_G_Y_Cali;
        PrivateData._uORB_MPU9250_G_Z -= PrivateData._flag_MPU9250_G_Z_Cali;
        PrivateData._uORB_MPU9250_A_X = PrivateData._uORB_MPU9250_A_X * PrivateData._flag_MPU9250_A_X_Scal - PrivateData._flag_MPU9250_A_X_Cali;
        PrivateData._uORB_MPU9250_A_Y = PrivateData._uORB_MPU9250_A_Y * PrivateData._flag_MPU9250_A_Y_Scal - PrivateData._flag_MPU9250_A_Y_Cali;
        PrivateData._uORB_MPU9250_A_Z = PrivateData._uORB_MPU9250_A_Z * PrivateData._flag_MPU9250_A_Z_Scal - PrivateData._flag_MPU9250_A_Z_Cali;
        float AccVibeFloorX = pt1FilterApply(&VibeFloorLPFX, (PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB));
        float AccVibeFloorY = pt1FilterApply(&VibeFloorLPFY, (PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB));
        float AccVibeFloorZ = pt1FilterApply(&VibeFloorLPFZ, (PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB));
        PrivateData._uORB_Accel_VIBE_X = pt1FilterApply(&VibeLPFX, (pow((((float)PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB) - AccVibeFloorX), 2)));
        PrivateData._uORB_Accel_VIBE_Y = pt1FilterApply(&VibeLPFY, (pow((((float)PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB) - AccVibeFloorY), 2)));
        PrivateData._uORB_Accel_VIBE_Z = pt1FilterApply(&VibeLPFZ, (pow((((float)PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB) - AccVibeFloorZ), 2)));
        //========================= //=========================
        switch (PrivateConfig.GyroFilterType)
        {
        case FilterLPFPT1:
            PrivateData._uORB_Gryo_Pitch = pt1FilterApply(&GryoFilterLPFX, ((float)PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB));
            PrivateData._uORB_Gryo__Roll = pt1FilterApply(&GryoFilterLPFY, ((float)PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB));
            PrivateData._uORB_Gryo___Yaw = pt1FilterApply(&GryoFilterLPFZ, ((float)PrivateData._uORB_MPU9250_G_Z / MPU9250_Gryo_LSB));
            break;
        case FilterLPFBiquad:
            PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterBLPFX, ((float)PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB));
            PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterBLPFY, ((float)PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB));
            PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterBLPFZ, ((float)PrivateData._uORB_MPU9250_G_Z / MPU9250_Gryo_LSB));
            break;
        }
        //
        if (PrivateConfig.GyroFilterNotchCutOff)
        {
            PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GyroNotchPitch, PrivateData._uORB_Gryo_Pitch);
            PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GyroNotch_Roll, PrivateData._uORB_Gryo__Roll);
            PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GyroNotch__Yaw, PrivateData._uORB_Gryo___Yaw);
        }
        if (PrivateConfig.DynamicNotchEnable)
        {
            if (GyroDynamicNotchReady)
            {
                int DT = (float)(1.f / (float)PrivateConfig.TargetFreqency) * 1000000.f;
                biquadFilterUpdate(&GryoFilterDynamicNotchX, PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[0], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                biquadFilterUpdate(&GryoFilterDynamicNotchY, PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[1], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                biquadFilterUpdate(&GryoFilterDynamicNotchZ, PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[2], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                GyroDynamicNotchReady = false;
            }
            PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterDynamicNotchY, PrivateData._uORB_Gryo__Roll);
            PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterDynamicNotchX, PrivateData._uORB_Gryo_Pitch);
            PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterDynamicNotchZ, PrivateData._uORB_Gryo___Yaw);
        }
        if (PrivateConfig.GyroDynamicAnalyse &&
            GyroDynamicNotchSampleCount == (PrivateConfig.TargetFreqency / 1000) &&
            PrivateConfig.TargetFreqency >= 1000)
        {
            IMUDynamicNotchUpdate();
            GyroDynamicNotchSampleCount = 0;
        }
        else
            GyroDynamicNotchSampleCount++;
        //
        switch (PrivateConfig.AccelFilterType)
        {
        case FilterLPFPT1:
            PrivateData._uORB_MPU9250_ADF_X = pt1FilterApply(&AccelFilterLPFX, ((float)PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB));
            PrivateData._uORB_MPU9250_ADF_Y = pt1FilterApply(&AccelFilterLPFY, ((float)PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB));
            PrivateData._uORB_MPU9250_ADF_Z = pt1FilterApply(&AccelFilterLPFZ, ((float)PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB));
            break;
        case FilterLPFBiquad:
            PrivateData._uORB_MPU9250_ADF_X = biquadFilterApply(&AccelFilterBLPFX, ((float)PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB));
            PrivateData._uORB_MPU9250_ADF_Y = biquadFilterApply(&AccelFilterBLPFY, ((float)PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB));
            PrivateData._uORB_MPU9250_ADF_Z = biquadFilterApply(&AccelFilterBLPFZ, ((float)PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB));
            break;
        }
        //========================= //=========================
        //TODO: caculate accelweight
        if ((PrivateData._uORB_MPU9250_A_Vector > (PrivateData._uORB_MPU9250_A_Static_Vector - MAX_ACC_NEARNESS)) &&
            (PrivateData._uORB_MPU9250_A_Vector < (PrivateData._uORB_MPU9250_A_Static_Vector + MAX_ACC_NEARNESS)))
            PrivateData.MPUMixTraditionBeta = PrivateConfig.GyroToAccelBeta;
        else
            PrivateData.MPUMixTraditionBeta = 0.f;
        AHRSSys->MadgwickSetAccelWeight(PrivateData.MPUMixTraditionBeta);
        AHRSSys->MadgwickAHRSupdateIMU(PrivateData._uORB_Gryo_Pitch, PrivateData._uORB_Gryo__Roll, PrivateData._uORB_Gryo___Yaw,
                                       (-1.f * PrivateData._uORB_MPU9250_ADF_X),
                                       (PrivateData._uORB_MPU9250_ADF_Y),
                                       (PrivateData._uORB_MPU9250_ADF_Z));
        AHRSSys->MadgwickAHRSGetQ(PrivateData._uORB_Raw_QuaternionQ[0],
                                  PrivateData._uORB_Raw_QuaternionQ[1],
                                  PrivateData._uORB_Raw_QuaternionQ[2],
                                  PrivateData._uORB_Raw_QuaternionQ[3]);
        AHRSSys->MadgwickComputeAngles(PrivateData._uORB_Real_Pitch, PrivateData._uORB_Real__Roll, PrivateData._uORB_Real___Yaw);
        PrivateData._uORB_Real__Roll *= 180.f / PI;
        PrivateData._uORB_Real_Pitch *= 180.f / PI;
        PrivateData._uORB_Real___Yaw *= 180.f / PI;
        //========================= //=========================
        PrivateData._uORB_MPU9250_A_Vector = sqrtf((PrivateData._uORB_MPU9250_ADF_X * PrivateData._uORB_MPU9250_ADF_X) +
                                                   (PrivateData._uORB_MPU9250_ADF_Y * PrivateData._uORB_MPU9250_ADF_Y) +
                                                   (PrivateData._uORB_MPU9250_ADF_Z * PrivateData._uORB_MPU9250_ADF_Z));
        PrivateData._uORB_Accel_Pitch = atan2((float)PrivateData._uORB_MPU9250_ADF_Y, PrivateData._uORB_MPU9250_ADF_Z) * 180.f / PI;
        PrivateData._uORB_Accel__Roll = atan2((float)PrivateData._uORB_MPU9250_ADF_X, PrivateData._uORB_MPU9250_ADF_Z) * 180.f / PI;
        //========================= //=========================
        PrivateData._uORB_MPU9250_Quaternion = Eigen::AngleAxisd((PrivateData._uORB_Real__Roll * (PI / 180.f)), Eigen::Vector3d::UnitZ()) *
                                               Eigen::AngleAxisd((-1.f * PrivateData._uORB_Real_Pitch * (PI / 180.f)), Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd((0 * (PI / 180.f)), Eigen::Vector3d::UnitX());

        PrivateData._uORB_MPU9250_RotationMatrix = PrivateData._uORB_MPU9250_Quaternion.normalized().toRotationMatrix();
        Eigen::Matrix<double, 1, 3> AccelRaw;
        AccelRaw << PrivateData._uORB_MPU9250_ADF_Z,
            PrivateData._uORB_MPU9250_ADF_X,
            PrivateData._uORB_MPU9250_ADF_Y;
        //========================= //=========================
        Eigen::Matrix<double, 1, 3> AccelStatic = AccelRaw * PrivateData._uORB_MPU9250_RotationMatrix;
        PrivateData._uORB_MPU9250_A_Static_Z = AccelStatic[0] - PrivateData._uORB_MPU9250_A_Static_Vector;
        PrivateData._uORB_MPU9250_A_Static_X = -1.f * AccelStatic[1];
        PrivateData._uORB_MPU9250_A_Static_Y = -1.f * AccelStatic[2];
        PrivateData._uORB_Acceleration_X = ((float)PrivateData._uORB_MPU9250_A_Static_X) * GravityAccel * 100.f;
        PrivateData._uORB_Acceleration_Y = ((float)PrivateData._uORB_MPU9250_A_Static_Y) * GravityAccel * 100.f;
        PrivateData._uORB_Acceleration_Z = ((float)PrivateData._uORB_MPU9250_A_Static_Z) * GravityAccel * 100.f;

        return PrivateData;
    }

    // This function set Total Angle to Accel Angle immediately , Require MPUSensorsDataGet() finish
    inline void ResetMPUMixAngle()
    {
        AHRSSys->MadgwickResetToAccel();
    }

private:
    inline void IMUSensorsDeviceInit()
    {
        double OutputSpeedCal = (MPU_250HZ_LPF_SPEED / (float)PrivateConfig.TargetFreqency) - 1.f;
        if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            MPU9250_fd = wiringPiSPISetup(PrivateConfig.MPUSPIChannel, MPU9250_SPI_Freq);
            MPU9250_SPI_Config[0] = 0x6b;
            MPU9250_SPI_Config[1] = 0x00;
            wiringPiSPIDataRW(PrivateConfig.MPUSPIChannel, MPU9250_SPI_Config, 2); //reset
            MPU9250_SPI_Config[0] = 0x1d;
            MPU9250_SPI_Config[1] = 0x03;                                          //FChoice 1, DLPF 3 , dlpf cut off 41hz
            wiringPiSPIDataRW(PrivateConfig.MPUSPIChannel, MPU9250_SPI_Config, 2); // Accel2
            MPU9250_SPI_Config[0] = 0x1c;
            MPU9250_SPI_Config[1] = 0x18;                                          //Full AccelScale +- 16g
            wiringPiSPIDataRW(PrivateConfig.MPUSPIChannel, MPU9250_SPI_Config, 2); // Accel
            MPU9250_SPI_Config[0] = 0x1b;
            MPU9250_SPI_Config[1] = 0x18;                                          // Full GyroScale +-2000dps, dlpf 250hz
            wiringPiSPIDataRW(PrivateConfig.MPUSPIChannel, MPU9250_SPI_Config, 2); // Gryo
            MPU9250_SPI_Config[0] = 0x1a;
            MPU9250_SPI_Config[1] = 0x00;                                          //DLPF_CFG is 000 , with Gyro dlpf is 250hz
            wiringPiSPIDataRW(PrivateConfig.MPUSPIChannel, MPU9250_SPI_Config, 2); //config
            MPU9250_SPI_Config[0] = 0x19;
            MPU9250_SPI_Config[1] = OutputSpeedCal;                                // 1khz / (1 + OutputSpeedCal) = 500hz; OutputSpeedCal is 2;
            wiringPiSPIDataRW(PrivateConfig.MPUSPIChannel, MPU9250_SPI_Config, 2); //DLPF's Sample rate's DIV , when > 250 hz lpf, not work
        }
        else if (PrivateConfig.MPUType == MPUTypeI2C)
        {
            MPU9250_fd = wiringPiI2CSetup(PrivateConfig.MPUI2CAddress);
            wiringPiI2CWriteReg8(MPU9250_fd, 107, 0x00);          //reset
            wiringPiI2CWriteReg8(MPU9250_fd, 29, 0x03);           //Accel
            wiringPiI2CWriteReg8(MPU9250_fd, 28, 0x18);           //Accel
            wiringPiI2CWriteReg8(MPU9250_fd, 27, 0x18);           //Gryo
            wiringPiI2CWriteReg8(MPU9250_fd, 26, 0x00);           //config
            wiringPiI2CWriteReg8(MPU9250_fd, 25, OutputSpeedCal); //DLPF's Sample rate's DIV
        }
    };

    inline void IMUSensorsDataRead()
    {
        if (PrivateConfig.MPUType == MPUTypeI2C)
        {
            Tmp_MPU9250_Buffer[0] = wiringPiI2CReadReg8(MPU9250_fd, 0x3B);
            Tmp_MPU9250_Buffer[1] = wiringPiI2CReadReg8(MPU9250_fd, 0x3C);
            PrivateData._uORB_MPU9250_A_X = (short)(Tmp_MPU9250_Buffer[0] << 8 | Tmp_MPU9250_Buffer[1]) * -1;
            Tmp_MPU9250_Buffer[2] = wiringPiI2CReadReg8(MPU9250_fd, 0x3D);
            Tmp_MPU9250_Buffer[3] = wiringPiI2CReadReg8(MPU9250_fd, 0x3E);
            PrivateData._uORB_MPU9250_A_Y = (short)(Tmp_MPU9250_Buffer[2] << 8 | Tmp_MPU9250_Buffer[3]);
            Tmp_MPU9250_Buffer[4] = wiringPiI2CReadReg8(MPU9250_fd, 0x3F);
            Tmp_MPU9250_Buffer[5] = wiringPiI2CReadReg8(MPU9250_fd, 0x40);
            PrivateData._uORB_MPU9250_A_Z = (short)(Tmp_MPU9250_Buffer[4] << 8 | Tmp_MPU9250_Buffer[5]);

            Tmp_MPU9250_Buffer[6] = wiringPiI2CReadReg8(MPU9250_fd, 0x43);
            Tmp_MPU9250_Buffer[7] = wiringPiI2CReadReg8(MPU9250_fd, 0x44);
            PrivateData._uORB_MPU9250_G_X = (short)(Tmp_MPU9250_Buffer[6] << 8 | Tmp_MPU9250_Buffer[7]);
            Tmp_MPU9250_Buffer[8] = wiringPiI2CReadReg8(MPU9250_fd, 0x45);
            Tmp_MPU9250_Buffer[9] = wiringPiI2CReadReg8(MPU9250_fd, 0x46);
            PrivateData._uORB_MPU9250_G_Y = (short)(Tmp_MPU9250_Buffer[8] << 8 | Tmp_MPU9250_Buffer[9]);
            Tmp_MPU9250_Buffer[10] = wiringPiI2CReadReg8(MPU9250_fd, 0x47);
            Tmp_MPU9250_Buffer[11] = wiringPiI2CReadReg8(MPU9250_fd, 0x48);
            PrivateData._uORB_MPU9250_G_Z = (short)(Tmp_MPU9250_Buffer[10] << 8 | Tmp_MPU9250_Buffer[11]);
        }
        else if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            Tmp_MPU9250_SPI_Buffer[0] = 0xBB;
            wiringPiSPIDataRW(PrivateConfig.MPUSPIChannel, Tmp_MPU9250_SPI_Buffer, 21);
            PrivateData._uORB_MPU9250_A_X = (short)((int)Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)Tmp_MPU9250_SPI_Buffer[2]) * -1;
            PrivateData._uORB_MPU9250_A_Y = (short)((int)Tmp_MPU9250_SPI_Buffer[3] << 8 | (int)Tmp_MPU9250_SPI_Buffer[4]);
            PrivateData._uORB_MPU9250_A_Z = (short)((int)Tmp_MPU9250_SPI_Buffer[5] << 8 | (int)Tmp_MPU9250_SPI_Buffer[6]);

            PrivateData._uORB_MPU9250_G_X = (short)((int)Tmp_MPU9250_SPI_Buffer[9] << 8 | (int)Tmp_MPU9250_SPI_Buffer[10]);
            PrivateData._uORB_MPU9250_G_Y = (short)((int)Tmp_MPU9250_SPI_Buffer[11] << 8 | (int)Tmp_MPU9250_SPI_Buffer[12]);
            PrivateData._uORB_MPU9250_G_Z = (short)((int)Tmp_MPU9250_SPI_Buffer[13] << 8 | (int)Tmp_MPU9250_SPI_Buffer[14]);

            PrivateData._uORB_MPU9250_M_X = (short)((int)Tmp_MPU9250_SPI_Buffer[16] << 8) | (int)Tmp_MPU9250_SPI_Buffer[15];
            PrivateData._uORB_MPU9250_M_Y = (short)((int)Tmp_MPU9250_SPI_Buffer[18] << 8) | (int)Tmp_MPU9250_SPI_Buffer[17];
            PrivateData._uORB_MPU9250_M_Z = (short)((int)Tmp_MPU9250_SPI_Buffer[20] << 8) | (int)Tmp_MPU9250_SPI_Buffer[19];
        }
    }

    //Collect GryoData 1000HZ and DownSample to 500hz by 1/2
    inline void IMUSensorFFTAnalyse()
    {
        FFTOverSample[0] += PrivateData._uORB_Gryo__Roll;
        FFTOverSample[1] += PrivateData._uORB_Gryo_Pitch;
        FFTOverSample[2] += PrivateData._uORB_Gryo___Yaw;
        FFTOverSampleCount++;
        //DownSample DataSheet To 500hz
        if (FFTOverSampleCount > (PrivateConfig.TargetFreqency / 1000))
        {
            FFTOverSampleCount = 0;
            FFTData[0][FFTCountDown].Re = FFTOverSample[0] / (float)(PrivateConfig.TargetFreqency / 1000);
            FFTData[1][FFTCountDown].Re = FFTOverSample[1] / (float)(PrivateConfig.TargetFreqency / 1000);
            FFTData[2][FFTCountDown].Re = FFTOverSample[2] / (float)(PrivateConfig.TargetFreqency / 1000);
            FFTData[0][FFTCountDown].Im = 0;
            FFTData[1][FFTCountDown].Im = 0;
            FFTData[2][FFTCountDown].Im = 0;
            //
            FFTOverSample[0] = 0;
            FFTOverSample[1] = 0;
            FFTOverSample[2] = 0;
            FFTCountDown++;
        }

        if (FFTCountDown > 16.f)
        {
        }
    };

    inline void IMUDynamicNotchUpdate(){

    };

    int MPU9250_fd;
    float MPU9250_Gryo_LSB = 16.4;    // +-2000dps
    float MPU9250_Accel_LSB = 2048.f; //+-16g
    int MPU9250_SPI_Freq = 10000000;
    unsigned char MPU9250_SPI_Config[20] = {0};
    unsigned char Tmp_MPU9250_Buffer[20] = {0};
    unsigned char Tmp_MPU9250_SPI_Buffer[20] = {0};
    MPUData PrivateData;
    MPUConfig PrivateConfig;
    MadgwickAHRS *AHRSSys;
    //CleanFlight Filter
    pt1Filter_t GryoFilterLPFX;
    pt1Filter_t GryoFilterLPFY;
    pt1Filter_t GryoFilterLPFZ;

    biquadFilter_t GryoFilterBLPFX;
    biquadFilter_t GryoFilterBLPFY;
    biquadFilter_t GryoFilterBLPFZ;

    pt1Filter_t AccelFilterLPFX;
    pt1Filter_t AccelFilterLPFY;
    pt1Filter_t AccelFilterLPFZ;
    biquadFilter_t AccelFilterBLPFX;
    biquadFilter_t AccelFilterBLPFY;
    biquadFilter_t AccelFilterBLPFZ;

    pt1Filter_t VibeFloorLPFX;
    pt1Filter_t VibeFloorLPFY;
    pt1Filter_t VibeFloorLPFZ;
    pt1Filter_t VibeLPFX;
    pt1Filter_t VibeLPFY;
    pt1Filter_t VibeLPFZ;

    biquadFilter_t GyroNotchPitch;
    biquadFilter_t GyroNotch_Roll;
    biquadFilter_t GyroNotch__Yaw;

    //Dynamic Filter
    int FFTCountDown = 0;
    int FFTAsixSampleCount = 0;
    int FFTOverSampleCount = 0;
    int GyroDynamicNotchSampleCount = 0;
    bool GyroDynamicNotchReady = false;

    complex FFTData[3][25];
    complex FFTDataTmp[3][25];
    float FFTOverSample[3] = {0};
    int GyroDynamicNotchCenterLast[3] = {250};
    biquadFilter_t GryoFilterDynamicNotchX;
    biquadFilter_t GryoFilterDynamicNotchY;
    biquadFilter_t GryoFilterDynamicNotchZ;
    biquadFilter_t GryoFilterDynamicFreq[3];
};