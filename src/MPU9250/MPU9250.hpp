#include <math.h>
#include <vector>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <iostream>

#include "filter.h"
#include "FFTPlugin.hpp"
#include "MadgwickAHRS.hpp"
#include "../_thirdparty/libeigen/Eigen/Dense"
#include "../_thirdparty/libeigen/Eigen/LU"

#ifdef MPUSPI_PIGPIO
#include "../_thirdparty/LinuxDriver/SPI/Drive_PIGPIO.h"
#elif MPUSPI_CUSTOM
#include "../_thirdparty/LinuxDriver/SPI/Drive_Custom.h"
#else
#include "../_thirdparty/LinuxDriver/SPI/Drive_LinuxSPI.h"
#endif

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
#define GravityAccel 9.80665
#define MPU_250HZ_LPF_SPEED 8000.f
#define MPU_LOWHZ_LPF_SPEED 1000.f

#define DYNAMIC_NOTCH_DEFAULT_CENTER_HZ 350.f
#define DYN_NOTCH_SMOOTH_FREQ_HZ 50.f
#define ACC_VIBE_FLOOR_FILT_HZ 5.f
#define ACC_VIBE_FILT_HZ 2.f
#define IMU_CALI_MAX_LOOP 500.0
#define MAX_ACC_NEARNESS 0.33 // 33% or G error soft-accepted (0.67-1.33G)
#define ACC_CLIPPING_THRESHOLD_G 7.9f
#define M_Ef 2.71828182845904523536f

#define GAXR 0
#define GAYP 1
#define GAZY 2
#define AAXN 0
#define AAYN 1
#define AAZN 2

#define DEG2RAD(x) (x * PI / 180.f)

struct MPUConfig
{
    int MPUType = MPUTypeSPI;
    const char *MPUSPIChannel = "/dev/spidev0.0";
    uint8_t MPUI2CAddress = 0x68;
    int MPU9250_SPI_Freq = 400000;
    float MPU_Flip_Pitch = 0;
    float MPU_Flip__Roll = 0;
    float MPU_Flip___Yaw = 0;
    //
    int TargetFreqency = 1000;
    float GyroToAccelBeta = 0.02;
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

    float FFTSampleBox[3][25] = {{0}};
    float _uORB_Gyro_Dynamic_NotchCenterHZ[3] = {350, 350, 350};
    Eigen::Matrix3d _uORB_MPU9250_RotationMatrix;
    Eigen::Quaternion<double> _uORB_MPU9250_Quaternion;

    int _uORB_MPU9250_IMUUpdateTime = 0;
    int _uORB_MPU9250_AccelCountDown = 0;
    int _uORB_MPU9250_CalibrationCountDown = 0;
};

class RPiMPU9250
{
public:
    inline RPiMPU9250(MPUConfig mpuConfig)
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        IMUstartuptime = (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))));

        LastUpdate = GetTimestamp();
        PrivateData._uORB_MPU9250_IMUUpdateTime = GetTimestamp();

        PrivateConfig = mpuConfig;
        GyroDynmiacNotchMinBox = (PrivateConfig.DynamicNotchMinFreq / FFTResolution) - 1;
        //Settings all Filter
        {
            int DT = (float)(1.f / (float)PrivateConfig.TargetFreqency) * 1000000;
            int ACCDT = (float)(1.f / (float)PrivateConfig.AccTargetFreqency) * 1000000;
            switch (PrivateConfig.GyroFilterType)
            {
            case FilterLPFPT1:
                pt1FilterInit(&GryoFilterLPF[GAXR], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPF[GAYP], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPF[GAZY], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                break;

            case FilterLPFBiquad:
                biquadFilterInitLPF(&GryoFilterBLPF[GAXR], PrivateConfig.GyroFilterCutOff, DT);
                biquadFilterInitLPF(&GryoFilterBLPF[GAYP], PrivateConfig.GyroFilterCutOff, DT);
                biquadFilterInitLPF(&GryoFilterBLPF[GAZY], PrivateConfig.GyroFilterCutOff, DT);
                break;
            }
            switch (PrivateConfig.GyroFilterTypeST2)
            {
            case FilterLPFPT1:
                pt1FilterInit(&GryoFilterLPFST2[GAXR], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPFST2[GAYP], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPFST2[GAZY], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
                break;

            case FilterLPFBiquad:
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAXR], PrivateConfig.GyroFilterCutOffST2, DT);
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAYP], PrivateConfig.GyroFilterCutOffST2, DT);
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAZY], PrivateConfig.GyroFilterCutOffST2, DT);
                break;
            }
            switch (PrivateConfig.AccelFilterType)
            {
            case FilterLPFPT1:
                pt1FilterInit(&AccelFilterLPF[AAXN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
                pt1FilterInit(&AccelFilterLPF[AAYN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
                pt1FilterInit(&AccelFilterLPF[AAZN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
                break;

            case FilterLPFBiquad:
                biquadFilterInitLPF(&AccelFilterBLPF[AAXN], PrivateConfig.AccelFilterCutOff, ACCDT);
                biquadFilterInitLPF(&AccelFilterBLPF[AAYN], PrivateConfig.AccelFilterCutOff, ACCDT);
                biquadFilterInitLPF(&AccelFilterBLPF[AAZN], PrivateConfig.AccelFilterCutOff, ACCDT);
                break;
            }

            if (PrivateConfig.GyroFilterNotchCutOff)
            {
                biquadFilterInitNotch(&GyroNotchFilter[GAXR], DT, PrivateConfig.GyroFilterNotchCenterFreq, PrivateConfig.GyroFilterNotchCutOff);
                biquadFilterInitNotch(&GyroNotchFilter[GAYP], DT, PrivateConfig.GyroFilterNotchCenterFreq, PrivateConfig.GyroFilterNotchCutOff);
                biquadFilterInitNotch(&GyroNotchFilter[GAZY], DT, PrivateConfig.GyroFilterNotchCenterFreq, PrivateConfig.GyroFilterNotchCutOff);
            }
            if (PrivateConfig.DynamicNotchEnable)
            {
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAXR], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAYP], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAZY], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInit(&GryoFilterDynamicNotch[GAXR], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
                biquadFilterInit(&GryoFilterDynamicNotch[GAYP], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
                biquadFilterInit(&GryoFilterDynamicNotch[GAZY], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
            }
            pt1FilterInit(&VibeFloorLPF[AAXN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeFloorLPF[AAYN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeFloorLPF[AAZN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAXN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAYN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAZN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
        }
        // MPUInit
        IMUSensorsDeviceInit();
        AHRSSys.reset(new MadgwickAHRS(PrivateConfig.GyroToAccelBeta, PrivateConfig.TargetFreqency));
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
        PrivateData._flag_MPU9250_A_TR_Cali = AccelCaliData[MPUAccelTRIM_Roll];
        PrivateData._flag_MPU9250_A_TP_Cali = AccelCaliData[MPUAccelTRIMPitch];

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
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }

        for (int cali_count = 0; cali_count < IMU_CALI_MAX_LOOP; cali_count++)
        {
            MPUSensorsDataGet();
            _Tmp_Accel_Static_Cali += PrivateData._uORB_MPU9250_A_Vector;
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }

        PrivateData._flag_MPU9250_G_X_Cali = _Tmp_Gryo_X_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._flag_MPU9250_G_Y_Cali = _Tmp_Gryo_Y_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._flag_MPU9250_G_Z_Cali = _Tmp_Gryo_Z_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._uORB_MPU9250_A_Static_Vector = _Tmp_Accel_Static_Cali / IMU_CALI_MAX_LOOP;
        return 0;
    };

    inline int MPUCalibrationOnline()
    {
        if (PrivateData._uORB_MPU9250_CalibrationCountDown == 0)
        {
            PrivateData._flag_MPU9250_G_X_Cali = 0;
            PrivateData._flag_MPU9250_G_Y_Cali = 0;
            PrivateData._flag_MPU9250_G_Z_Cali = 0;
            PrivateData._uORB_MPU9250_A_Static_Vector = 1.0;
            _Tmp_Gryo_X_Cali_ON = 0;
            _Tmp_Gryo_Y_Cali_ON = 0;
            _Tmp_Gryo_Z_Cali_ON = 0;
            _Tmp_Accel_Static_Cali_ON = 0;
        }

        _Tmp_Gryo_X_Cali_ON += PrivateData._uORB_MPU9250_G_X;
        _Tmp_Gryo_Y_Cali_ON += PrivateData._uORB_MPU9250_G_Y;
        _Tmp_Gryo_Z_Cali_ON += PrivateData._uORB_MPU9250_G_Z;
        _Tmp_Accel_Static_Cali_ON += PrivateData._uORB_MPU9250_A_Vector;

        if (PrivateData._uORB_MPU9250_CalibrationCountDown >= IMU_CALI_MAX_LOOP)
        {
            PrivateData._flag_MPU9250_G_X_Cali = _Tmp_Gryo_X_Cali_ON / IMU_CALI_MAX_LOOP;
            PrivateData._flag_MPU9250_G_Y_Cali = _Tmp_Gryo_Y_Cali_ON / IMU_CALI_MAX_LOOP;
            PrivateData._flag_MPU9250_G_Z_Cali = _Tmp_Gryo_Z_Cali_ON / IMU_CALI_MAX_LOOP;
            PrivateData._uORB_MPU9250_A_Static_Vector = _Tmp_Accel_Static_Cali_ON / IMU_CALI_MAX_LOOP;
            PrivateData._uORB_MPU9250_CalibrationCountDown = 0;
            return 0;
        }
        PrivateData._uORB_MPU9250_CalibrationCountDown++;
        return 1;
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

    inline void MPUSensorApplyAHRS(int mx, int my, int mz, bool enabled)
    {
        _Tmp_AHRS_MAG_X = mx;
        _Tmp_AHRS_MAG_Y = my;
        _Tmp_AHRS_MAG_Z = mz;
        AHRSEnable = enabled;
    };
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
        //========================= //=========================Gyro Filter
        {
            switch (PrivateConfig.GyroFilterType)
            {
            case FilterLPFPT1:
                if (PrivateConfig.GyroFilterCutOff)
                {
                    PrivateData._uORB_Gryo__Roll = pt1FilterApply(&GryoFilterLPF[GAXR], ((float)PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB));
                    PrivateData._uORB_Gryo_Pitch = pt1FilterApply(&GryoFilterLPF[GAYP], ((float)PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB));
                    PrivateData._uORB_Gryo___Yaw = pt1FilterApply(&GryoFilterLPF[GAZY], ((float)PrivateData._uORB_MPU9250_G_Z / MPU9250_Gryo_LSB));
                }
                else
                {
                    PrivateData._uORB_Gryo__Roll = ((float)PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB);
                    PrivateData._uORB_Gryo_Pitch = ((float)PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB);
                    PrivateData._uORB_Gryo___Yaw = ((float)PrivateData._uORB_MPU9250_G_Z / MPU9250_Gryo_LSB);
                }
                break;
            case FilterLPFBiquad:
                if (PrivateConfig.GyroFilterCutOff)
                {
                    PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterBLPF[GAXR], ((float)PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB));
                    PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterBLPF[GAYP], ((float)PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB));
                    PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterBLPF[GAZY], ((float)PrivateData._uORB_MPU9250_G_Z / MPU9250_Gryo_LSB));
                }
                else
                {
                    PrivateData._uORB_Gryo__Roll = ((float)PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB);
                    PrivateData._uORB_Gryo_Pitch = ((float)PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB);
                    PrivateData._uORB_Gryo___Yaw = ((float)PrivateData._uORB_MPU9250_G_Z / MPU9250_Gryo_LSB);
                }
                break;
            }
            switch (PrivateConfig.GyroFilterTypeST2)
            {
            case FilterLPFPT1:
                if (PrivateConfig.GyroFilterCutOffST2)
                {
                    PrivateData._uORB_Gryo__Roll = pt1FilterApply(&GryoFilterLPFST2[GAXR], PrivateData._uORB_Gryo__Roll);
                    PrivateData._uORB_Gryo_Pitch = pt1FilterApply(&GryoFilterLPFST2[GAYP], PrivateData._uORB_Gryo_Pitch);
                    PrivateData._uORB_Gryo___Yaw = pt1FilterApply(&GryoFilterLPFST2[GAZY], PrivateData._uORB_Gryo___Yaw);
                }
                break;
            case FilterLPFBiquad:
                if (PrivateConfig.GyroFilterCutOffST2)
                {
                    PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterBLPFST2[GAXR], PrivateData._uORB_Gryo__Roll);
                    PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterBLPFST2[GAYP], PrivateData._uORB_Gryo_Pitch);
                    PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterBLPFST2[GAZY], PrivateData._uORB_Gryo___Yaw);
                }
                break;
            }

            if (PrivateConfig.GyroFilterNotchCutOff)
            {
                PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GyroNotchFilter[GAXR], PrivateData._uORB_Gryo_Pitch);
                PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GyroNotchFilter[GAYP], PrivateData._uORB_Gryo__Roll);
                PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GyroNotchFilter[GAZY], PrivateData._uORB_Gryo___Yaw);
            }
        }
        //========================= //=========================Dynamic Anlyisc
        {

            if (PrivateConfig.TargetFreqency >= 1000)
            {
                if (PrivateConfig.GyroDynamicAnalyse && GyroDynamicFFTSampleCount == (PrivateConfig.TargetFreqency / 1000))
                {
                    IMUSensorFFTAnalyse();
                    GyroDynamicFFTSampleCount = 0;
                    //
                    if (PrivateConfig.DynamicNotchEnable)
                    {
                        if (GyroDynamicNotchReady)
                        {
                            IMUDynamicNotchUpdate();
                            int DT = (float)(1.f / (float)PrivateConfig.TargetFreqency) * 1000000.f;
                            biquadFilterUpdate(&GryoFilterDynamicNotch[GAXR], PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[GAXR], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                            biquadFilterUpdate(&GryoFilterDynamicNotch[GAYP], PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[GAYP], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                            biquadFilterUpdate(&GryoFilterDynamicNotch[GAZY], PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[GAZY], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                            GyroDynamicNotchReady = false;
                        }
                    }
                }
                else
                    GyroDynamicFFTSampleCount++;
                //
                if (PrivateConfig.DynamicNotchEnable)
                {
                    PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterDynamicNotch[GAXR], PrivateData._uORB_Gryo__Roll);
                    PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterDynamicNotch[GAYP], PrivateData._uORB_Gryo_Pitch);
                    PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterDynamicNotch[GAZY], PrivateData._uORB_Gryo___Yaw);
                }
            }
        }
        //========================= //=========================Accel Filter
        {
            if (PrivateData._uORB_MPU9250_AccelCountDown == 1)
            {
                float AccVibeFloorX = pt1FilterApply(&VibeFloorLPF[AAXN], (PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB));
                float AccVibeFloorY = pt1FilterApply(&VibeFloorLPF[AAYN], (PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB));
                float AccVibeFloorZ = pt1FilterApply(&VibeFloorLPF[AAZN], (PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB));
                PrivateData._uORB_Accel_VIBE_X = pt1FilterApply(&VibeLPF[AAXN], (pow((((float)PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB) - AccVibeFloorX), 2)));
                PrivateData._uORB_Accel_VIBE_Y = pt1FilterApply(&VibeLPF[AAYN], (pow((((float)PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB) - AccVibeFloorY), 2)));
                PrivateData._uORB_Accel_VIBE_Z = pt1FilterApply(&VibeLPF[AAZN], (pow((((float)PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB) - AccVibeFloorZ), 2)));
                //
                if (PrivateConfig.AccelFilterCutOff)
                {
                    switch (PrivateConfig.AccelFilterType)
                    {
                    case FilterLPFPT1:
                        PrivateData._uORB_MPU9250_ADF_X = pt1FilterApply(&AccelFilterLPF[AAXN], ((float)PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB));
                        PrivateData._uORB_MPU9250_ADF_Y = pt1FilterApply(&AccelFilterLPF[AAYN], ((float)PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB));
                        PrivateData._uORB_MPU9250_ADF_Z = pt1FilterApply(&AccelFilterLPF[AAZN], ((float)PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB));
                        break;
                    case FilterLPFBiquad:
                        PrivateData._uORB_MPU9250_ADF_X = biquadFilterApply(&AccelFilterBLPF[AAXN], ((float)PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB));
                        PrivateData._uORB_MPU9250_ADF_Y = biquadFilterApply(&AccelFilterBLPF[AAYN], ((float)PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB));
                        PrivateData._uORB_MPU9250_ADF_Z = biquadFilterApply(&AccelFilterBLPF[AAZN], ((float)PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB));
                        break;
                    }
                }
                else
                {
                    PrivateData._uORB_MPU9250_ADF_X = PrivateData._uORB_MPU9250_A_X / MPU9250_Accel_LSB;
                    PrivateData._uORB_MPU9250_ADF_Y = PrivateData._uORB_MPU9250_A_Y / MPU9250_Accel_LSB;
                    PrivateData._uORB_MPU9250_ADF_Z = PrivateData._uORB_MPU9250_A_Z / MPU9250_Accel_LSB;
                }
                //
                if (abs(PrivateData._uORB_MPU9250_ADF_X) > ACC_CLIPPING_THRESHOLD_G ||
                    abs(PrivateData._uORB_MPU9250_ADF_Y) > ACC_CLIPPING_THRESHOLD_G ||
                    abs(PrivateData._uORB_MPU9250_ADF_Z) > ACC_CLIPPING_THRESHOLD_G)
                    PrivateData._uORB_MPU9250_ACC_Clipped = true;
                else
                    PrivateData._uORB_MPU9250_ACC_Clipped = false;
                //
                PrivateData._uORB_MPU9250_A_Vector = sqrtf((PrivateData._uORB_MPU9250_ADF_X * PrivateData._uORB_MPU9250_ADF_X) +
                                                           (PrivateData._uORB_MPU9250_ADF_Y * PrivateData._uORB_MPU9250_ADF_Y) +
                                                           (PrivateData._uORB_MPU9250_ADF_Z * PrivateData._uORB_MPU9250_ADF_Z));
                PrivateData._uORB_Accel_Pitch = atan2((float)PrivateData._uORB_MPU9250_ADF_X, PrivateData._uORB_MPU9250_ADF_Z) * 180.f / PI;
                PrivateData._uORB_Accel__Roll = atan2((float)PrivateData._uORB_MPU9250_ADF_Y, PrivateData._uORB_MPU9250_ADF_Z) * 180.f / PI;
            }
        }
        //========================= //=========================AHRS Update
        {
            if ((PrivateData._uORB_MPU9250_A_Vector > (PrivateData._uORB_MPU9250_A_Static_Vector - MAX_ACC_NEARNESS)) &&
                (PrivateData._uORB_MPU9250_A_Vector < (PrivateData._uORB_MPU9250_A_Static_Vector + MAX_ACC_NEARNESS)))
                PrivateData.MPUMixTraditionBeta = PrivateConfig.GyroToAccelBeta;
            else
                PrivateData.MPUMixTraditionBeta = 0.f;

            if (!AHRSEnable)
                AHRSSys->MadgwickAHRSIMUApply(PrivateData._uORB_Gryo__Roll, PrivateData._uORB_Gryo_Pitch, PrivateData._uORB_Gryo___Yaw,
                                              (PrivateData._uORB_MPU9250_ADF_X),
                                              (PrivateData._uORB_MPU9250_ADF_Y),
                                              (PrivateData._uORB_MPU9250_ADF_Z),
                                              ((float)PrivateData._uORB_MPU9250_IMUUpdateTime * 1e-6f));
            else
                AHRSSys->MadgwickAHRSApply(PrivateData._uORB_Gryo__Roll, PrivateData._uORB_Gryo_Pitch, PrivateData._uORB_Gryo___Yaw,
                                           (PrivateData._uORB_MPU9250_ADF_X),
                                           (PrivateData._uORB_MPU9250_ADF_Y),
                                           (PrivateData._uORB_MPU9250_ADF_Z),
                                           (_Tmp_AHRS_MAG_X),
                                           (_Tmp_AHRS_MAG_Y),
                                           (_Tmp_AHRS_MAG_Z),
                                           ((float)PrivateData._uORB_MPU9250_IMUUpdateTime * 1e-6f));

            AHRSSys->MadgwickSetAccelWeight(PrivateData.MPUMixTraditionBeta);
            AHRSSys->MadgwickAHRSGetQ(PrivateData._uORB_Raw_QuaternionQ[0],
                                      PrivateData._uORB_Raw_QuaternionQ[1],
                                      PrivateData._uORB_Raw_QuaternionQ[2],
                                      PrivateData._uORB_Raw_QuaternionQ[3]);
            AHRSSys->MadgwickComputeAngles(PrivateData._uORB_Real__Roll, PrivateData._uORB_Real_Pitch, PrivateData._uORB_Real___Yaw);

            PrivateData._uORB_Real__Roll *= 180.f / PI;
            PrivateData._uORB_Real_Pitch *= 180.f / PI;
            PrivateData._uORB_Real___Yaw *= 180.f / PI;
            PrivateData._uORB_Real___Yaw = PrivateData._uORB_Real___Yaw > 0 ? 360 - PrivateData._uORB_Real___Yaw : PrivateData._uORB_Real___Yaw;
            PrivateData._uORB_Real___Yaw = PrivateData._uORB_Real___Yaw < 0 ? -1 * PrivateData._uORB_Real___Yaw : PrivateData._uORB_Real___Yaw;
            //
            PrivateData._uORB_Real__Roll += PrivateData._flag_MPU9250_A_TR_Cali;
            PrivateData._uORB_Real_Pitch += PrivateData._flag_MPU9250_A_TP_Cali;
        }
        //========================= //=========================Navigation update
        {
            if (PrivateData._uORB_MPU9250_AccelCountDown == 1)
            {
                PrivateData._uORB_MPU9250_Quaternion = Eigen::AngleAxisd(((PrivateData._uORB_Real__Roll - PrivateData._flag_MPU9250_A_TR_Cali) * (PI / 180.f)), Eigen::Vector3d::UnitZ()) *
                                                       Eigen::AngleAxisd(((PrivateData._uORB_Real_Pitch - PrivateData._flag_MPU9250_A_TP_Cali) * (PI / 180.f)), Eigen::Vector3d::UnitY()) *
                                                       Eigen::AngleAxisd((0 * (PI / 180.f)), Eigen::Vector3d::UnitX());

                PrivateData._uORB_MPU9250_RotationMatrix = PrivateData._uORB_MPU9250_Quaternion.normalized().toRotationMatrix();
                Eigen::Matrix<double, 1, 3> AccelRaw;
                AccelRaw << PrivateData._uORB_MPU9250_ADF_Z,
                    PrivateData._uORB_MPU9250_ADF_Y,
                    PrivateData._uORB_MPU9250_ADF_X;

                Eigen::Matrix<double, 1, 3> AccelStatic = AccelRaw * PrivateData._uORB_MPU9250_RotationMatrix;
                PrivateData._uORB_MPU9250_A_Static_Z = AccelStatic[0] - PrivateData._uORB_MPU9250_A_Static_Vector;
                PrivateData._uORB_MPU9250_A_Static_X = -1.f * AccelStatic[1];
                PrivateData._uORB_MPU9250_A_Static_Y = -1.f * AccelStatic[2];
                PrivateData._uORB_Acceleration_X = ((float)PrivateData._uORB_MPU9250_A_Static_X) * GravityAccel * 100.f;
                PrivateData._uORB_Acceleration_Y = ((float)PrivateData._uORB_MPU9250_A_Static_Y) * GravityAccel * 100.f;
                PrivateData._uORB_Acceleration_Z = ((float)PrivateData._uORB_MPU9250_A_Static_Z) * GravityAccel * 100.f;
            }
        }
        return PrivateData;
    }

    // This function set Total Angle to Accel Angle immediately , Require MPUSensorsDataGet() finish
    inline void ResetMPUMixAngle()
    {
        AHRSSys->MadgwickResetToAccel();
    }

    inline ~RPiMPU9250()
    {
        AHRSSys.reset();
        _s_spiClose(MPU9250_fd);
    };

private:
    inline void IMUSensorsDeviceInit()
    {
        double OutputSpeedCal = (MPU_250HZ_LPF_SPEED / (float)PrivateConfig.TargetFreqency) - 1.f;
        if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            MPU9250_fd = _s_spiOpen(PrivateConfig.MPUSPIChannel, PrivateConfig.MPU9250_SPI_Freq, 0);
            if (MPU9250_fd < 0)
                throw - 2;

            char MPU9250_SPI_Config_WHOAMI[2] = {0xf5, 0x00};
            _s_spiXfer(MPU9250_fd, MPU9250_SPI_Config_WHOAMI, MPU9250_SPI_Config_WHOAMI, PrivateConfig.MPU9250_SPI_Freq, 2);
            PrivateData.DeviceType = MPU9250_SPI_Config_WHOAMI[1];

            char MPU9250_SPI_Config_RESET[2] = {0x6b, 0x80};
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_RESET, PrivateConfig.MPU9250_SPI_Freq, 2); // reset
            usleep(500);
            char MPU9250_SPI_Config_RESET2[2] = {0x68, 0x07};
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_RESET2, PrivateConfig.MPU9250_SPI_Freq, 2); // BIT_GYRO | BIT_ACC | BIT_TEMP reset
            usleep(500);
            char MPU9250_SPI_Config_RESET3[2] = {0x6b, 0x00};
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_RESET3, PrivateConfig.MPU9250_SPI_Freq, 2); // reset
            usleep(500);
            char MPU9250_SPI_Config_RESET4[2] = {0x6b, 0x01};
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_RESET4, PrivateConfig.MPU9250_SPI_Freq, 2); // reset
            usleep(1000);

            char MPU9250_SPI_Config_ALPF[2] = {0x1d, 0x03};                                      // FChoice 1, DLPF 3 , dlpf cut off 44.8hz for accel
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_ALPF, PrivateConfig.MPU9250_SPI_Freq, 2); // Accel2
            usleep(15);
            char MPU9250_SPI_Config_Acce[2] = {0x1c, 0x18};                                      // Full AccelScale +- 16g
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_Acce, PrivateConfig.MPU9250_SPI_Freq, 2); // Accel
            usleep(15);
            char MPU9250_SPI_Config_Gyro[2] = {0x1b, 0x18};                                      // Full GyroScale +-2000dps, dlpf 250hz
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_Gyro, PrivateConfig.MPU9250_SPI_Freq, 2); // Gryo
            usleep(15);
            char MPU9250_SPI_Config_GLPF[2] = {0x1a, 0x00};                                      // DLPF_CFG is 000 , with Gyro dlpf is 250hz
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_GLPF, PrivateConfig.MPU9250_SPI_Freq, 2); // config
            usleep(15);
            // char MPU9250_SPI_Config_SIMP[2] = {0x19, 0x04};   // 1khz / (1 + OutputSpeedCal) = 500hz; OutputSpeedCal is 2;
            // _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_SIMP, 2); // DLPF's Sample rate's DIV , when > 250 hz lpf, not work
            // usleep(15);
            char MPU9250_SPI_Config_INTC[2] = {0x37, 0x22};
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_INTC, PrivateConfig.MPU9250_SPI_Freq, 2);
            usleep(500);
            char MPU9250_SPI_Config_INTE[2] = {0x38, 0x01};
            _s_spiWrite(MPU9250_fd, MPU9250_SPI_Config_INTE, PrivateConfig.MPU9250_SPI_Freq, 2);
            usleep(500);
        }
        else if (PrivateConfig.MPUType == MPUTypeI2C)
        {
        }
    };

    inline void IMUSensorsDataRead()
    {
        if (PrivateConfig.MPUType == MPUTypeI2C)
        {
            //
        }
        else if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            char Tmp_MPU9250_SPI_BufferX[2] = {0};
            Tmp_MPU9250_SPI_BufferX[0] = 0xBA;
            _s_spiXfer(MPU9250_fd, Tmp_MPU9250_SPI_BufferX, Tmp_MPU9250_SPI_BufferX, PrivateConfig.MPU9250_SPI_Freq, 2);
            if (Tmp_MPU9250_SPI_BufferX[1] & 0x01)
            {
                PrivateData._uORB_MPU9250_IMUUpdateTime = GetTimestamp() - LastUpdate;
                LastUpdate = GetTimestamp();

                char Tmp_MPU9250_SPI_Buffer[8] = {0};
                Tmp_MPU9250_SPI_Buffer[0] = 0xBB;
                if (PrivateData._uORB_MPU9250_AccelCountDown >= (PrivateConfig.TargetFreqency / PrivateConfig.AccTargetFreqency))
                {
                    _s_spiXfer(MPU9250_fd, Tmp_MPU9250_SPI_Buffer, Tmp_MPU9250_SPI_Buffer, PrivateConfig.MPU9250_SPI_Freq, 8);
                    int Tmp_AX = (short)((int)Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)Tmp_MPU9250_SPI_Buffer[2]);
                    int Tmp_AY = (short)((int)Tmp_MPU9250_SPI_Buffer[3] << 8 | (int)Tmp_MPU9250_SPI_Buffer[4]);
                    int Tmp_AZ = (short)((int)Tmp_MPU9250_SPI_Buffer[5] << 8 | (int)Tmp_MPU9250_SPI_Buffer[6]);
                    // Step 1: rotate Yaw
                    int Tmp_A2X = Tmp_AX * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_AY * sin(DEG2RAD((PrivateConfig.MPU_Flip___Yaw)));
                    int Tmp_A2Y = Tmp_AY * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_AX * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip___Yaw)));
                    // Step 2: rotate Pitch
                    int Tmp_A3X = Tmp_A2X * cos(DEG2RAD(PrivateConfig.MPU_Flip_Pitch)) + Tmp_AZ * sin(DEG2RAD((PrivateConfig.MPU_Flip_Pitch)));
                    int Tmp_A3Z = Tmp_AZ * cos(DEG2RAD((PrivateConfig.MPU_Flip_Pitch))) + Tmp_A2X * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip_Pitch)));
                    // Step 3: rotate Roll
                    PrivateData._uORB_MPU9250_A_Y = Tmp_A2Y * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_A3Z * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_MPU9250_A_Z = Tmp_A3Z * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_A2Y * sin(DEG2RAD((PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_MPU9250_A_X = Tmp_A3X;
                    //
                    PrivateData._uORB_MPU9250_AccelCountDown = 0;
                }
                PrivateData._uORB_MPU9250_AccelCountDown++;

                {
                    char Tmp_MPU9250_SPI_GBuffer[8] = {0};
                    Tmp_MPU9250_SPI_GBuffer[0] = 0xC3;
                    _s_spiXfer(MPU9250_fd, Tmp_MPU9250_SPI_GBuffer, Tmp_MPU9250_SPI_GBuffer, PrivateConfig.MPU9250_SPI_Freq, 8);
                    int Tmp_GX = (short)((int)Tmp_MPU9250_SPI_GBuffer[1] << 8 | (int)Tmp_MPU9250_SPI_GBuffer[2]);
                    int Tmp_GY = (short)((int)Tmp_MPU9250_SPI_GBuffer[3] << 8 | (int)Tmp_MPU9250_SPI_GBuffer[4]);
                    int Tmp_GZ = (short)((int)Tmp_MPU9250_SPI_GBuffer[5] << 8 | (int)Tmp_MPU9250_SPI_GBuffer[6]);
                    // Step 1: rotate Yaw
                    int Tmp_G2X = Tmp_GX * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_GY * sin(DEG2RAD((PrivateConfig.MPU_Flip___Yaw)));
                    int Tmp_G2Y = Tmp_GY * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_GX * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip___Yaw)));
                    // Step 2: rotate Pitch
                    int Tmp_G3X = Tmp_G2X * cos(DEG2RAD(PrivateConfig.MPU_Flip_Pitch)) + Tmp_GZ * sin(DEG2RAD((PrivateConfig.MPU_Flip_Pitch)));
                    int Tmp_G3Z = Tmp_GZ * cos(DEG2RAD((PrivateConfig.MPU_Flip_Pitch))) + Tmp_G2X * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip_Pitch)));
                    // Step 3: rotate Roll
                    PrivateData._uORB_MPU9250_G_Y = Tmp_G2Y * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_G3Z * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_MPU9250_G_Z = Tmp_G3Z * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_G2Y * sin(DEG2RAD((PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_MPU9250_G_X = Tmp_G3X;
                    //
                }
            }
        }
    }

    //Collect GryoData 1000HZ and DownSample to 500hz by 1/2
    inline void IMUSensorFFTAnalyse()
    {
        if (FFTCountDown < 16)
        {
            FFTOverSample[0] += PrivateData._uORB_Gryo__Roll;
            FFTOverSample[1] += PrivateData._uORB_Gryo_Pitch;
            FFTOverSample[2] += PrivateData._uORB_Gryo___Yaw;
            FFTOverSampleCount++;
            //DownSample DataSheet To 500hz
            if (FFTOverSampleCount >= (PrivateConfig.TargetFreqency / 1000))
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
        }
        // FFT caculate using 3 loop frame
        if (FFTCountDown >= 16)
        {
            fft(FFTData[GyroDynamicFFTCaculateCount], 16, FFTDataTmp[GyroDynamicFFTCaculateCount]);
            for (size_t i = 0; i < 16; i++)
                PrivateData.FFTSampleBox[GyroDynamicFFTCaculateCount][i] =
                    sqrt((FFTData[GyroDynamicFFTCaculateCount][i].Re * FFTData[GyroDynamicFFTCaculateCount][i].Re +
                          FFTData[GyroDynamicFFTCaculateCount][i].Im * FFTData[GyroDynamicFFTCaculateCount][i].Im));
            GyroDynamicFFTCaculateCount--;
        }

        if (FFTCountDown >= 16 && GyroDynamicFFTCaculateCount < 0)
        {
            FFTCountDown = 0;
            GyroDynamicFFTCaculateCount = 2;
            GyroDynamicNotchReady = true;
        }
    };

    inline void IMUDynamicNotchUpdate()
    {
        for (size_t x = 0; x < 3; x++)
        {
            //This Part is from betaflight/INAV
            {
                bool fftIncreased = false;
                float dataMax = 0;
                uint8_t binStart = 0;
                uint8_t binMax = 0;
                //for bins after initial decline, identify start bin and max bin
                for (int i = GyroDynmiacNotchMinBox; i < 9; i++)
                {
                    if (fftIncreased || (PrivateData.FFTSampleBox[x][i] > PrivateData.FFTSampleBox[x][i - 1]))
                    {
                        if (!fftIncreased)
                        {
                            binStart = i; // first up-step bin
                            fftIncreased = true;
                        }
                        if (PrivateData.FFTSampleBox[x][i] > dataMax)
                        {
                            dataMax = PrivateData.FFTSampleBox[x][i];
                            binMax = i; // tallest bin
                        }
                    }
                }
                // accumulate fftSum and fftWeightedSum from peak bin, and shoulder bins either side of peak
                float cubedData = PrivateData.FFTSampleBox[x][binMax] * PrivateData.FFTSampleBox[x][binMax] * PrivateData.FFTSampleBox[x][binMax];
                float fftSum = cubedData;
                float fftWeightedSum = cubedData * (binMax + 1);
                // accumulate upper shoulder
                for (int i = binMax; i < 9 - 1; i++)
                {
                    if (PrivateData.FFTSampleBox[x][i] > PrivateData.FFTSampleBox[x][i + 1])
                    {
                        cubedData = PrivateData.FFTSampleBox[x][i] * PrivateData.FFTSampleBox[x][i] * PrivateData.FFTSampleBox[x][i];
                        fftSum += cubedData;
                        fftWeightedSum += cubedData * (i + 1);
                    }
                    else
                    {
                        break;
                    }
                }
                // accumulate lower shoulder
                for (int i = binMax; i > binStart + 1; i--)
                {
                    if (PrivateData.FFTSampleBox[x][i] > PrivateData.FFTSampleBox[x][i - 1])
                    {
                        cubedData = PrivateData.FFTSampleBox[x][i] * PrivateData.FFTSampleBox[x][i] * PrivateData.FFTSampleBox[x][i];
                        fftSum += cubedData;
                        fftWeightedSum += cubedData * (i + 1);
                    }
                    else
                    {
                        break;
                    }
                }
                // get weighted center of relevant frequency range (this way we have a better resolution than 31.25Hz)
                float centerFreq = DYNAMIC_NOTCH_DEFAULT_CENTER_HZ;
                float fftMeanIndex = 0;
                // idx was shifted by 1 to start at 1, not 0
                if (fftSum > 0)
                {
                    fftMeanIndex = (fftWeightedSum / fftSum) - 1;
                    // the index points at the center frequency of each bin so index 0 is actually 16.125Hz
                    centerFreq = fftMeanIndex * FFTResolution;
                }
                else
                {
                    centerFreq = GyroDynamicNotchCenterLast[x];
                }
                centerFreq = centerFreq < PrivateConfig.DynamicNotchMinFreq ? PrivateConfig.DynamicNotchMinFreq : centerFreq;
                PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[x] = biquadFilterApply(&GryoFilterDynamicFreq[x], centerFreq);
                GyroDynamicNotchCenterLast[x] = PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[x];
            }
        }
    };

    inline int GetTimestamp()
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        return (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))) - IMUstartuptime);
    }

    int IMUstartuptime = 0;

    int MPU9250_fd;
    float MPU9250_Gryo_LSB = MPU9250_GYRO_LSB;   // +-2000dps
    float MPU9250_Accel_LSB = MPU9250_ACCEL_LSB; //+-16g
    char Tmp_MPU9250_Buffer[20] = {0};
    bool AHRSEnable = false;
    int LastUpdate = 0;
    MPUData PrivateData;
    MPUConfig PrivateConfig;
    std::unique_ptr<MadgwickAHRS> AHRSSys;
    //
    int _Tmp_AHRS_MAG_X = 0;
    int _Tmp_AHRS_MAG_Y = 0;
    int _Tmp_AHRS_MAG_Z = 0;
    double _Tmp_Gryo_X_Cali_ON = 0;
    double _Tmp_Gryo_Y_Cali_ON = 0;
    double _Tmp_Gryo_Z_Cali_ON = 0;
    double _Tmp_Accel_Static_Cali_ON = 0;
    //CleanFlight Filter
    pt1Filter_t VibeLPF[3];
    pt1Filter_t VibeFloorLPF[3];
    pt1Filter_t GryoFilterLPF[3];
    pt1Filter_t GryoFilterLPFST2[3];
    pt1Filter_t AccelFilterLPF[3];
    biquadFilter_t GryoFilterBLPF[3];
    biquadFilter_t GryoFilterBLPFST2[3];
    biquadFilter_t AccelFilterBLPF[3];
    biquadFilter_t GyroNotchFilter[3];
    //Dynamic Filter
    int FFTCountDown = 0;
    int FFTAsixSampleCount = 0;
    int FFTOverSampleCount = 0;
    int GyroDynamicFFTSampleCount = 0;
    int GyroDynamicFFTCaculateCount = 0;
    bool GyroDynamicNotchReady = false;
    complex FFTData[3][25];
    complex FFTDataTmp[3][25];
    float FFTOverSample[3] = {0};
    int GyroDynmiacNotchMinBox = 2;
    int GyroDynamicNotchCenterLast[3] = {350, 350, 350};
    biquadFilter_t GryoFilterDynamicNotch[3];
    biquadFilter_t GryoFilterDynamicFreq[3];
};