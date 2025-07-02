#pragma once
#include <math.h>
#include <vector>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <iostream>
#include <stdexcept>
//
#include "filter.h"
#include "FFTPlugin.hpp"
#include "MadgwickAHRS.hpp"
#include "_thirdparty/libeigen/Eigen/Dense"
#include "_thirdparty/libeigen/Eigen/LU"
//
#include "MPU.hpp"
#include "MPU9250/MPU9250.hpp"
#include "ICM20602/ICM20602.hpp"
#include "ICM42605/ICM42605.hpp"
#include <mutex>
//
#include <ostream>

#ifdef MPUSPI_PIGPIO
#include "_thirdparty/LinuxDriver/SPI/Drive_PIGPIO.h"
#elif MPUSPI_CUSTOM
#include "_thirdparty/LinuxDriver/SPI/Drive_Custom.h"
#else
#include "_thirdparty/LinuxDriver/SPI/Drive_LinuxSPI.h"
#endif

/**
 * @brief Enhanced MPU9250/ICM20602/ICM42605 IMU processing class with Earth frame accelerometer conversion
 * 
 * This class provides comprehensive IMU data processing including:
 * - Gyroscope and accelerometer data fusion using Madgwick AHRS
 * - Advanced filtering (PT1, Biquad, Notch, Dynamic Notch)
 * - FFT-based vibration analysis
 * - Earth frame accelerometer conversion with:
 *   * Quaternion-based coordinate transformation
 *   * Multiple coordinate system support (NED, ENU, NWU)
 *   * Total acceleration (including gravity) in cm/s²
 *   * Pure acceleration (without gravity) available via GetPureAcceleration()
 * 
 * Usage example:
 * @code
 * MPUConfig config;
 * config.TargetFreqency = 1000;
 * config.AccTargetFreqency = 1000;
 * 
 * RPiMPU9250 imu(config);
 * imu.MPUCalibration(calibration_data);
 * 
 * while (running) {
 *     MPUData data = imu.MPUSensorsDataGet();
 *     
 *     // Access total Earth frame acceleration (cm/s²) - includes gravity
 *     // X: North, Y: West, Z: Up (NWU coordinate system)
 *     float accel_north = data._uORB_Acceleration_X;  // cm/s² (includes gravity)
 *     float accel_west = data._uORB_Acceleration_Y;   // cm/s² (includes gravity)
 *     float accel_up = data._uORB_Acceleration_Z;     // cm/s² (includes gravity, ~980 when stationary)
 *     
 *     // Get pure acceleration (without gravity) - should be ~0 when stationary
 *     float ax_pure, ay_pure, az_pure;
 *     imu.GetPureAcceleration(ax_pure, ay_pure, az_pure);
 *     
 *     // For different coordinate systems, use:
 *     // imu.ConvertAccelToEarthFrameCustom(0); // NED (North-East-Down)
 *     // imu.ConvertAccelToEarthFrameCustom(1); // ENU (East-North-Up)
 *     // imu.ConvertAccelToEarthFrameCustom(2); // NWU (North-West-Up) - Default
 * }
 * @endcode
 */
class RPiMPU9250
{
public:
    auto GetMPUTypeDetected() const & -> const SensorType & { return PrivateConfig.GyroScope; }

    inline RPiMPU9250(MPUConfig mpuConfig)
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        IMUstartuptime = (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))));

        LastUpdate = GetTimestamp();
        PrivateData._uORB_MPU9250_IMUUpdateTime = GetTimestamp();

        PrivateConfig = mpuConfig;
        GyroDynmiacNotchMinBox = (PrivateConfig.DynamicNotchMinFreq / FFTResolution) - 1;

        // Settings all Filter
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
            if (PrivateConfig.AccelFilterNotchCutOff)
            {
                biquadFilterInitNotch(&AccelNotchFilter[AAXN], ACCDT, PrivateConfig.AccelFilterNotchCenterFreq, PrivateConfig.AccelFilterNotchCutOff);
                biquadFilterInitNotch(&AccelNotchFilter[AAYN], ACCDT, PrivateConfig.AccelFilterNotchCenterFreq, PrivateConfig.AccelFilterNotchCutOff);
                biquadFilterInitNotch(&AccelNotchFilter[AAZN], ACCDT, PrivateConfig.AccelFilterNotchCenterFreq, PrivateConfig.AccelFilterNotchCutOff);
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

        if (PrivateConfig.GyroDynamicAnalyse)
        {
            for (size_t i = 0; i < 3; i++)
            {
                FFTData.push_back(std::vector<float>(FFTBINCOUNT));
                PrivateData.FFTSampleBox.push_back(std::vector<float>(FFTBINCOUNT));
            }
            //
            FFTWINDOW = std::vector<float>(FFTBINCOUNT);
            VecBuildBlackmanHarrisWindow(FFTWINDOW.data(), FFTWINDOW.size());
        }

        // MPUInit
        {
            Sensor_fd = _s_spiOpen(PrivateConfig.GyroSPIChannel, PrivateConfig.MPU9250_SPI_Freq, 0);
            if (Sensor_fd < 0)
                throw std::invalid_argument("SPI configuration failed");

            if (PrivateConfig.GyroScope == SensorType::AUTO)
            {
                uint8_t SPI_Config_WHOAMI[2] = {0xf5, 0x00};
                _s_spiXfer(Sensor_fd, SPI_Config_WHOAMI, SPI_Config_WHOAMI, PrivateConfig.MPU9250_SPI_Freq, 2);
                PrivateData.DeviceType = SPI_Config_WHOAMI[1];
                if (PrivateData.DeviceType == SensorType::ICM20602)
                    PrivateConfig.GyroScope = SensorType::ICM20602;
                else if (PrivateData.DeviceType == SensorType::ICM42605)
                    PrivateConfig.GyroScope = SensorType::ICM42605;
                else if (PrivateData.DeviceType == SensorType::MPU9250)
                    PrivateConfig.GyroScope = SensorType::MPU9250;
                else if (PrivateData.DeviceType == SensorType::MPU9255)
                    PrivateConfig.GyroScope = SensorType::MPU9250;
                else if (PrivateData.DeviceType == SensorType::MPU6500)
                    PrivateConfig.GyroScope = SensorType::MPU9250; // FIXME: deal as 9250

                std::string s;
                if (PrivateConfig.GyroScope == SensorType::AUTO)
                    throw std::range_error(
                        static_cast<std::ostringstream &&>(
                            (std::ostringstream() << "Can't find Gyro type :" << std::hex << (int)SPI_Config_WHOAMI[1]) << std::dec << "\n")
                            .str());
            }

            switch (PrivateConfig.GyroScope)
            {
            case SensorType::MPU9250:
                MPU9250Init(PrivateConfig, PrivateData, Sensor_fd);
                break;
            case SensorType::ICM20602:
                ICM20602Init(PrivateConfig, PrivateData, Sensor_fd);
                break;
            case SensorType::ICM42605:
                ICM42605Init(PrivateConfig, PrivateData, Sensor_fd);
                break;
            }
        }
        AHRSSys.reset(new MadgwickAHRS(PrivateConfig.GyroToAccelBeta, PrivateConfig.TargetFreqency));
        AHRSSys->MadgwickSetMagWeight(PrivateConfig.MagToYawBeta);
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

        int caliRequest = PrivateConfig.TargetFreqency / 4;
        for (int cali_count = 0; cali_count < caliRequest; cali_count++)
        {
            MPUSensorsDataGet();
            ResetMPUMixAngle();
            _Tmp_Gryo_X_Cali += PrivateData._uORB_MPU9250_G_X;
            _Tmp_Gryo_Y_Cali += PrivateData._uORB_MPU9250_G_Y;
            _Tmp_Gryo_Z_Cali += PrivateData._uORB_MPU9250_G_Z;

            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }

        for (int cali_count = 0; cali_count < caliRequest; cali_count++)
        {
            MPUSensorsDataGet();
            _Tmp_Accel_Static_Cali += PrivateData._uORB_MPU9250_A_Vector;
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }
        PrivateData._flag_MPU9250_G_X_Cali = _Tmp_Gryo_X_Cali / caliRequest;
        PrivateData._flag_MPU9250_G_Y_Cali = _Tmp_Gryo_Y_Cali / caliRequest;
        PrivateData._flag_MPU9250_G_Z_Cali = _Tmp_Gryo_Z_Cali / caliRequest;
        PrivateData._uORB_MPU9250_A_Static_Vector = _Tmp_Accel_Static_Cali / caliRequest;
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

        int caliRequest = PrivateConfig.TargetFreqency / 4;
        if (PrivateData._uORB_MPU9250_CalibrationCountDown >= caliRequest)
        {
            PrivateData._flag_MPU9250_G_X_Cali = _Tmp_Gryo_X_Cali_ON / caliRequest;
            PrivateData._flag_MPU9250_G_Y_Cali = _Tmp_Gryo_Y_Cali_ON / caliRequest;
            PrivateData._flag_MPU9250_G_Z_Cali = _Tmp_Gryo_Z_Cali_ON / caliRequest;
            PrivateData._uORB_MPU9250_A_Static_Vector = _Tmp_Accel_Static_Cali_ON / caliRequest;
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
        float calicounting = PrivateConfig.AccTargetFreqency * 2;
        for (int cali_count = 0; cali_count < calicounting; cali_count++)
        {
            IMUSensorsDataRead();
            switch (AccelCaliAction)
            {
            case MPUAccelNoseUp:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_X;
                break;
            case MPUAccelNoseDown:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_X;
                break;
            case MPUAccelNoseRight:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_Y;
                break;
            case MPUAccelNoseLeft:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_Y;
                break;
            case MPUAccelNoseTop:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_Z;
                break;
            case MPUAccelNoseRev:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU9250_A_Z;
                break;
            }
            usleep((int)(1.f / (float)PrivateConfig.AccTargetFreqency * 1000000.f));
        }
        if (AccelCaliAction == MPUAccelCaliGet)
        {
            AccelCaliData[MPUAccelNoseUp] /= calicounting;
            AccelCaliData[MPUAccelNoseDown] /= calicounting;
            AccelCaliData[MPUAccelNoseRight] /= calicounting;
            AccelCaliData[MPUAccelNoseLeft] /= calicounting;
            AccelCaliData[MPUAccelNoseTop] /= calicounting;
            AccelCaliData[MPUAccelNoseRev] /= calicounting;
            AccelCaliData[MPUAccelScalX] = std::abs(MPU9250_Accel_LSB / ((AccelCaliData[MPUAccelNoseUp] - AccelCaliData[MPUAccelNoseDown]) / 2.f));
            AccelCaliData[MPUAccelScalY] = std::abs(MPU9250_Accel_LSB / ((AccelCaliData[MPUAccelNoseLeft] - AccelCaliData[MPUAccelNoseRight]) / 2.f));
            AccelCaliData[MPUAccelScalZ] = std::abs(MPU9250_Accel_LSB / ((AccelCaliData[MPUAccelNoseTop] - AccelCaliData[MPUAccelNoseRev]) / 2.f));
            AccelCaliData[MPUAccelCaliX] = ((AccelCaliData[MPUAccelNoseUp] + AccelCaliData[MPUAccelNoseDown]) / 2.f) * AccelCaliData[MPUAccelScalX];
            AccelCaliData[MPUAccelCaliY] = ((AccelCaliData[MPUAccelNoseRight] + AccelCaliData[MPUAccelNoseLeft]) / 2.f) * AccelCaliData[MPUAccelScalY];
            AccelCaliData[MPUAccelCaliZ] = ((AccelCaliData[MPUAccelNoseTop] + AccelCaliData[MPUAccelNoseRev]) / 2.f) * AccelCaliData[MPUAccelScalZ];
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

        PrivateData._uORB_MPU9250_AC_X = PrivateData._uORB_MPU9250_A_X * PrivateData._flag_MPU9250_A_X_Scal - PrivateData._flag_MPU9250_A_X_Cali;
        PrivateData._uORB_MPU9250_AC_Y = PrivateData._uORB_MPU9250_A_Y * PrivateData._flag_MPU9250_A_Y_Scal - PrivateData._flag_MPU9250_A_Y_Cali;
        PrivateData._uORB_MPU9250_AC_Z = PrivateData._uORB_MPU9250_A_Z * PrivateData._flag_MPU9250_A_Z_Scal - PrivateData._flag_MPU9250_A_Z_Cali;

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
                            // biquadFilterUpdate(&GryoFilterDynamicNotch[GAXR], PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[GAXR], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                            // biquadFilterUpdate(&GryoFilterDynamicNotch[GAYP], PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[GAYP], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                            // biquadFilterUpdate(&GryoFilterDynamicNotch[GAZY], PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[GAZY], DT, PrivateConfig.DynamicNotchQ, FILTER_NOTCH);
                            GyroDynamicNotchReady = false;
                        }
                    }
                }
                else
                    GyroDynamicFFTSampleCount++;
                //
                if (PrivateConfig.DynamicNotchEnable)
                {
                    // PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterDynamicNotch[GAXR], PrivateData._uORB_Gryo__Roll);
                    // PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterDynamicNotch[GAYP], PrivateData._uORB_Gryo_Pitch);
                    // PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterDynamicNotch[GAZY], PrivateData._uORB_Gryo___Yaw);
                }
            }
        }
        //========================= //=========================Accel Filter
        {
            if (PrivateData._uORB_MPU9250_AccelCountDown == 1)
            {
                float AccVibeFloorX = pt1FilterApply(&VibeFloorLPF[AAXN], (PrivateData._uORB_MPU9250_AC_X / MPU9250_Accel_LSB));
                float AccVibeFloorY = pt1FilterApply(&VibeFloorLPF[AAYN], (PrivateData._uORB_MPU9250_AC_Y / MPU9250_Accel_LSB));
                float AccVibeFloorZ = pt1FilterApply(&VibeFloorLPF[AAZN], (PrivateData._uORB_MPU9250_AC_Z / MPU9250_Accel_LSB));
                PrivateData._uORB_Accel_VIBE_X = pt1FilterApply(&VibeLPF[AAXN], (pow((((float)PrivateData._uORB_MPU9250_AC_X / MPU9250_Accel_LSB) - AccVibeFloorX), 2)));
                PrivateData._uORB_Accel_VIBE_Y = pt1FilterApply(&VibeLPF[AAYN], (pow((((float)PrivateData._uORB_MPU9250_AC_Y / MPU9250_Accel_LSB) - AccVibeFloorY), 2)));
                PrivateData._uORB_Accel_VIBE_Z = pt1FilterApply(&VibeLPF[AAZN], (pow((((float)PrivateData._uORB_MPU9250_AC_Z / MPU9250_Accel_LSB) - AccVibeFloorZ), 2)));
                PrivateData._uORB_Accel_VIBE = sqrt(PrivateData._uORB_Accel_VIBE_X + PrivateData._uORB_Accel_VIBE_Y + PrivateData._uORB_Accel_VIBE_Z);
                //
                if (PrivateConfig.AccelFilterCutOff)
                {
                    switch (PrivateConfig.AccelFilterType)
                    {
                    case FilterLPFPT1:
                        PrivateData._uORB_MPU9250_ADF_X = pt1FilterApply(&AccelFilterLPF[AAXN], ((float)PrivateData._uORB_MPU9250_AC_X / MPU9250_Accel_LSB));
                        PrivateData._uORB_MPU9250_ADF_Y = pt1FilterApply(&AccelFilterLPF[AAYN], ((float)PrivateData._uORB_MPU9250_AC_Y / MPU9250_Accel_LSB));
                        PrivateData._uORB_MPU9250_ADF_Z = pt1FilterApply(&AccelFilterLPF[AAZN], ((float)PrivateData._uORB_MPU9250_AC_Z / MPU9250_Accel_LSB));
                        break;
                    case FilterLPFBiquad:
                        PrivateData._uORB_MPU9250_ADF_X = biquadFilterApply(&AccelFilterBLPF[AAXN], ((float)PrivateData._uORB_MPU9250_AC_X / MPU9250_Accel_LSB));
                        PrivateData._uORB_MPU9250_ADF_Y = biquadFilterApply(&AccelFilterBLPF[AAYN], ((float)PrivateData._uORB_MPU9250_AC_Y / MPU9250_Accel_LSB));
                        PrivateData._uORB_MPU9250_ADF_Z = biquadFilterApply(&AccelFilterBLPF[AAZN], ((float)PrivateData._uORB_MPU9250_AC_Z / MPU9250_Accel_LSB));
                        break;
                    }

                    if (PrivateConfig.AccelFilterNotchCutOff)
                    {
                        PrivateData._uORB_MPU9250_ADF_X = biquadFilterApply(&AccelNotchFilter[AAXN], PrivateData._uORB_MPU9250_ADF_X);
                        PrivateData._uORB_MPU9250_ADF_Y = biquadFilterApply(&AccelNotchFilter[AAYN], PrivateData._uORB_MPU9250_ADF_Y);
                        PrivateData._uORB_MPU9250_ADF_Z = biquadFilterApply(&AccelNotchFilter[AAZN], PrivateData._uORB_MPU9250_ADF_Z);
                    }
                }
                else
                {
                    PrivateData._uORB_MPU9250_ADF_X = PrivateData._uORB_MPU9250_AC_X / MPU9250_Accel_LSB;
                    PrivateData._uORB_MPU9250_ADF_Y = PrivateData._uORB_MPU9250_AC_Y / MPU9250_Accel_LSB;
                    PrivateData._uORB_MPU9250_ADF_Z = PrivateData._uORB_MPU9250_AC_Z / MPU9250_Accel_LSB;
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
                PrivateData._uORB_Accel_Pitch = -1 * atan2((float)PrivateData._uORB_MPU9250_ADF_X, PrivateData._uORB_MPU9250_ADF_Z) * 180.f / PI;
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
            // Convert accelerometer to Earth frame using quaternion-based transformation
            ConvertAccelToEarthFrame();
        }
        return PrivateData;
    }

    // This function set Total Angle to Accel Angle immediately , Require MPUSensorsDataGet() finish
    inline void ResetMPUMixAngle()
    {
        AHRSSys->MadgwickResetToAccel();
    }

    inline void ResetMAGMixYaw()
    {
        AHRSSys->MadgwickResetToMag();
    }

    // Convert accelerometer data to Earth frame
    inline void ConvertAccelToEarthFrame()
    {
        if (PrivateData._uORB_MPU9250_AccelCountDown == 1)
        {
            // Get the current quaternion from AHRS (more accurate than Euler angles)
            float q0 = PrivateData._uORB_Raw_QuaternionQ[0];
            float q1 = PrivateData._uORB_Raw_QuaternionQ[1];
            float q2 = PrivateData._uORB_Raw_QuaternionQ[2];
            float q3 = PrivateData._uORB_Raw_QuaternionQ[3];
            
            // Normalize quaternion
            float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q0 /= norm;
            q1 /= norm;
            q2 /= norm;
            q3 /= norm;
            
            // Accelerometer readings in sensor frame (already filtered)
            float ax = PrivateData._uORB_MPU9250_ADF_X;
            float ay = PrivateData._uORB_MPU9250_ADF_Y;
            float az = PrivateData._uORB_MPU9250_ADF_Z;
            
            // Debug: Print raw accelerometer values
            static int debug_counter = 0;
            if (debug_enabled && debug_counter++ % 100 == 0) { // Print every 100th sample
                std::cout << "Raw Accel (g): X=" << ax << " Y=" << ay << " Z=" << az << std::endl;
                std::cout << "Quaternion: q0=" << q0 << " q1=" << q1 << " q2=" << q2 << " q3=" << q3 << std::endl;
            }
            
            // Convert to Earth frame using quaternion rotation
            // Earth frame: X North, Y West, Z Up
            float ax_earth = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * ax +
                           (2 * (q1*q2 - q0*q3)) * ay +
                           (2 * (q1*q3 + q0*q2)) * az;
                           
            float ay_earth = (2 * (q1*q2 + q0*q3)) * ax +
                           (q0*q0 - q1*q1 + q2*q2 - q3*q3) * ay +
                           (2 * (q2*q3 - q0*q1)) * az;
                           
            float az_earth = (2 * (q1*q3 - q0*q2)) * ax +
                           (2 * (q2*q3 + q0*q1)) * ay +
                           (q0*q0 - q1*q1 - q2*q2 + q3*q3) * az;
            
            // Debug: Print Earth frame values before unit conversion
            if (debug_enabled && debug_counter % 100 == 0) {
                std::cout << "Earth Frame (g): X=" << ax_earth << " Y=" << ay_earth << " Z=" << az_earth << std::endl;
            }
            
            // Store Earth frame acceleration (with gravity still included)
            PrivateData._uORB_MPU9250_A_Static_X = ax_earth;
            PrivateData._uORB_MPU9250_A_Static_Y = ay_earth;
            PrivateData._uORB_MPU9250_A_Static_Z = az_earth;
            
            // Convert to cm/s² units (1 m/s² = 100 cm/s²)
            PrivateData._uORB_Acceleration_X = ax_earth * GravityAccel * 100.0f;
            PrivateData._uORB_Acceleration_Y = ay_earth * GravityAccel * 100.0f;
            PrivateData._uORB_Acceleration_Z = az_earth * GravityAccel * 100.0f;
            
            // Debug: Print final values
            if (debug_enabled && debug_counter % 100 == 0) {
                std::cout << "Final (cm/s²): X=" << PrivateData._uORB_Acceleration_X 
                         << " Y=" << PrivateData._uORB_Acceleration_Y 
                         << " Z=" << PrivateData._uORB_Acceleration_Z << std::endl;
                std::cout << "GravityAccel=" << GravityAccel << std::endl;
                std::cout << "---" << std::endl;
            }
        }
    }

    // Convert accelerometer to Earth frame with custom coordinate system
    // frame_type: 0 = NED (North-East-Down), 1 = ENU (East-North-Up), 2 = NWU (North-West-Up)
    inline void ConvertAccelToEarthFrameCustom(int frame_type = 0)
    {
        if (PrivateData._uORB_MPU9250_AccelCountDown == 1)
        {
            // Get the current quaternion from AHRS
            float q0 = PrivateData._uORB_Raw_QuaternionQ[0];
            float q1 = PrivateData._uORB_Raw_QuaternionQ[1];
            float q2 = PrivateData._uORB_Raw_QuaternionQ[2];
            float q3 = PrivateData._uORB_Raw_QuaternionQ[3];
            
            // Normalize quaternion
            float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q0 /= norm;
            q1 /= norm;
            q2 /= norm;
            q3 /= norm;
            
            // Accelerometer readings in sensor frame
            float ax = PrivateData._uORB_MPU9250_ADF_X;
            float ay = PrivateData._uORB_MPU9250_ADF_Y;
            float az = PrivateData._uORB_MPU9250_ADF_Z;
            
            float ax_earth, ay_earth, az_earth;
            
            switch (frame_type) {
                case 0: // NED (North-East-Down)
                    ax_earth = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * ax +
                              (2 * (q1*q2 - q0*q3)) * ay +
                              (2 * (q1*q3 + q0*q2)) * az;
                    ay_earth = (2 * (q1*q2 + q0*q3)) * ax +
                              (q0*q0 - q1*q1 + q2*q2 - q3*q3) * ay +
                              (2 * (q2*q3 - q0*q1)) * az;
                    az_earth = (2 * (q1*q3 - q0*q2)) * ax +
                              (2 * (q2*q3 + q0*q1)) * ay +
                              (q0*q0 - q1*q1 - q2*q2 + q3*q3) * az;
                    break;
                    
                case 1: // ENU (East-North-Up)
                    ax_earth = (2 * (q1*q2 + q0*q3)) * ax +
                              (q0*q0 - q1*q1 + q2*q2 - q3*q3) * ay +
                              (2 * (q2*q3 - q0*q1)) * az;
                    ay_earth = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * ax +
                              (2 * (q1*q2 - q0*q3)) * ay +
                              (2 * (q1*q3 + q0*q2)) * az;
                    az_earth = (2 * (q1*q3 - q0*q2)) * ax +
                              (2 * (q2*q3 + q0*q1)) * ay +
                              (q0*q0 - q1*q1 - q2*q2 + q3*q3) * az;
                    break;
                    
                case 2: // NWU (North-West-Up) - Default
                default:
                    ax_earth = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * ax +
                              (2 * (q1*q2 - q0*q3)) * ay +
                              (2 * (q1*q3 + q0*q2)) * az;
                    ay_earth = (2 * (q1*q2 + q0*q3)) * ax +
                              (q0*q0 - q1*q1 + q2*q2 - q3*q3) * ay +
                              (2 * (q2*q3 - q0*q1)) * az;
                    az_earth = (2 * (q1*q3 - q0*q2)) * ax +
                              (2 * (q2*q3 + q0*q1)) * ay +
                              (q0*q0 - q1*q1 - q2*q2 + q3*q3) * az;
                    break;
            }
            
            // Store Earth frame acceleration
            PrivateData._uORB_MPU9250_A_Static_X = ax_earth;
            PrivateData._uORB_MPU9250_A_Static_Y = ay_earth;
            PrivateData._uORB_MPU9250_A_Static_Z = az_earth;
            
            // Convert to cm/s² units (1 m/s² = 100 cm/s²)
            PrivateData._uORB_Acceleration_X = ax_earth * GravityAccel * 100.0f;
            PrivateData._uORB_Acceleration_Y = ay_earth * GravityAccel * 100.0f;
            PrivateData._uORB_Acceleration_Z = az_earth * GravityAccel * 100.0f;
        }
    }

    // Get pure acceleration (without gravity) in Earth frame
    inline void GetPureAcceleration(float& ax_pure, float& ay_pure, float& az_pure)
    {
        // Get the current quaternion from AHRS
        float q0 = PrivateData._uORB_Raw_QuaternionQ[0];
        float q1 = PrivateData._uORB_Raw_QuaternionQ[1];
        float q2 = PrivateData._uORB_Raw_QuaternionQ[2];
        float q3 = PrivateData._uORB_Raw_QuaternionQ[3];
        
        // Normalize quaternion
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
        
        // Accelerometer readings in sensor frame
        float ax = PrivateData._uORB_MPU9250_ADF_X;
        float ay = PrivateData._uORB_MPU9250_ADF_Y;
        float az = PrivateData._uORB_MPU9250_ADF_Z;
        
        // Convert to Earth frame using quaternion rotation (NWU system)
        float ax_earth = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * ax +
                       (2 * (q1*q2 - q0*q3)) * ay +
                       (2 * (q1*q3 + q0*q2)) * az;
                       
        float ay_earth = (2 * (q1*q2 + q0*q3)) * ax +
                       (q0*q0 - q1*q1 + q2*q2 - q3*q3) * ay +
                       (2 * (q2*q3 - q0*q1)) * az;
                       
        float az_earth = (2 * (q1*q3 - q0*q2)) * ax +
                       (2 * (q2*q3 + q0*q1)) * ay +
                       (q0*q0 - q1*q1 - q2*q2 + q3*q3) * az;
        
        // Remove gravity component (gravity points down in NWU system)
        az_earth -= 1.0; // Remove 1g from Z component
        
        // Convert to cm/s² units
        ax_pure = ax_earth * GravityAccel * 100.0f;
        ay_pure = ay_earth * GravityAccel * 100.0f;
        az_pure = az_earth * GravityAccel * 100.0f;
    }

    // Get raw accelerometer values for debugging
    inline void GetRawAccelerometerValues(float& ax_raw, float& ay_raw, float& az_raw)
    {
        ax_raw = PrivateData._uORB_MPU9250_ADF_X;
        ay_raw = PrivateData._uORB_MPU9250_ADF_Y;
        az_raw = PrivateData._uORB_MPU9250_ADF_Z;
    }

    // Check if accelerometer is properly calibrated
    inline bool IsAccelerometerCalibrated()
    {
        // Check if calibration values are reasonable
        if (PrivateData._flag_MPU9250_A_X_Scal < 0.5f || PrivateData._flag_MPU9250_A_X_Scal > 2.0f) return false;
        if (PrivateData._flag_MPU9250_A_Y_Scal < 0.5f || PrivateData._flag_MPU9250_A_Y_Scal > 2.0f) return false;
        if (PrivateData._flag_MPU9250_A_Z_Scal < 0.5f || PrivateData._flag_MPU9250_A_Z_Scal > 2.0f) return false;
        
        // Check if static vector is reasonable (should be close to 1.0g when stationary)
        if (PrivateData._uORB_MPU9250_A_Static_Vector < 0.5f || PrivateData._uORB_MPU9250_A_Static_Vector > 1.5f) return false;
        
        return true;
    }

    // Enable/disable debug output
    inline void SetDebugOutput(bool enable)
    {
        debug_enabled = enable;
    }

    // Get debug information as string
    inline std::string GetDebugInfo()
    {
        std::ostringstream oss;
        oss << "Raw Accel (g): X=" << PrivateData._uORB_MPU9250_ADF_X 
            << " Y=" << PrivateData._uORB_MPU9250_ADF_Y 
            << " Z=" << PrivateData._uORB_MPU9250_ADF_Z << std::endl;
        oss << "Static Vector: " << PrivateData._uORB_MPU9250_A_Static_Vector << std::endl;
        oss << "Calibration: X_scale=" << PrivateData._flag_MPU9250_A_X_Scal 
            << " Y_scale=" << PrivateData._flag_MPU9250_A_Y_Scal 
            << " Z_scale=" << PrivateData._flag_MPU9250_A_Z_Scal << std::endl;
        oss << "Final (cm/s²): X=" << PrivateData._uORB_Acceleration_X 
            << " Y=" << PrivateData._uORB_Acceleration_Y 
            << " Z=" << PrivateData._uORB_Acceleration_Z << std::endl;
        return oss.str();
    }

    inline ~RPiMPU9250()
    {
        AHRSSys.reset();
        _s_spiClose(Sensor_fd);
    };

private:
    inline void IMUSensorsDataRead()
    {
        deviceLock.lock();
        if (PrivateConfig.MPUType == MPUTypeI2C)
        {
            //
        }
        else if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            PrivateData._uORB_MPU9250_IMUUpdateTime = GetTimestamp() - LastUpdate;
            LastUpdate = GetTimestamp();
            int Six_Axis[6] = {0};

            // TODO: should handling ACC and gyro, they not in same freq
            switch (PrivateConfig.GyroScope)
            {
            case SensorType::MPU9250:
                MPU9250DataSPIRead(Sensor_fd, Six_Axis, PrivateConfig.MPU9250_SPI_Freq);
                break;
            case SensorType::ICM20602:
                ICM20602DataSPIRead(Sensor_fd, Six_Axis, PrivateConfig.MPU9250_SPI_Freq);
                break;
            case SensorType::ICM42605:
                ICM42605DataSPIRead(Sensor_fd, Six_Axis, PrivateConfig.MPU9250_SPI_Freq);
                break;
            }
            if (PrivateData._uORB_MPU9250_AccelCountDown >= (PrivateConfig.TargetFreqency / PrivateConfig.AccTargetFreqency))
            {
                int Tmp_AX = Six_Axis[0];
                int Tmp_AY = Six_Axis[1];
                int Tmp_AZ = Six_Axis[2];
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
                int Tmp_GX = Six_Axis[3];
                int Tmp_GY = Six_Axis[4];
                int Tmp_GZ = Six_Axis[5];
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
        deviceLock.unlock();
    }

    // Collect GryoData 1000HZ and DownSample to 500hz by 1/2
    inline void IMUSensorFFTAnalyse()
    {
        if (FFTCountDown < 16)
        {
            FFTOverSample[0] += PrivateData._uORB_Gryo__Roll;
            FFTOverSample[1] += PrivateData._uORB_Gryo_Pitch;
            FFTOverSample[2] += PrivateData._uORB_Gryo___Yaw;
            FFTOverSampleCount++;
            // DownSample DataSheet To 500hz
            if (FFTOverSampleCount >= (PrivateConfig.TargetFreqency / 1000))
            {
                FFTOverSampleCount = 0;
                FFTData[0][FFTCountDown] = FFTOverSample[0] / (float)(PrivateConfig.TargetFreqency / 1000);
                FFTData[1][FFTCountDown] = FFTOverSample[1] / (float)(PrivateConfig.TargetFreqency / 1000);
                FFTData[2][FFTCountDown] = FFTOverSample[2] / (float)(PrivateConfig.TargetFreqency / 1000);
                //
                FFTOverSample[0] = 0;
                FFTOverSample[1] = 0;
                FFTOverSample[2] = 0;
                FFTCountDown++;
            }
        }
        // FFT caculate using 3 loop frame
        if (FFTCountDown >= FFTBINCOUNT)
        {
            PrivateData.FFTSampleBox[GyroDynamicFFTCaculateCount] = FFTFrequencyAnalyzer<float>(FFTData[GyroDynamicFFTCaculateCount], FFTWINDOW);
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
            // This Part is from betaflight/INAV
            {
                bool fftIncreased = false;
                float dataMax = 0;
                uint8_t binStart = 0;
                uint8_t binMax = 0;
                // for bins after initial decline, identify start bin and max bin
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
                    // centerFreq = fftMeanIndex * FFTResolution;
                    centerFreq = freqPeak(PrivateData.FFTSampleBox[x].data(), PrivateData.FFTSampleBox[x].size(), fftMeanIndex, 500);
                    PrivateData.fftindexs[x] = fftMeanIndex;
                }
                else
                {
                    centerFreq = GyroDynamicNotchCenterLast[x];
                }
                centerFreq = centerFreq < PrivateConfig.DynamicNotchMinFreq ? PrivateConfig.DynamicNotchMinFreq : centerFreq;
                PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[x] = biquadFilterApply(&GryoFilterDynamicFreq[x], centerFreq);
                GyroDynamicNotchCenterLast[x] = PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[x];
            }
            // float centerFreq = DYNAMIC_NOTCH_DEFAULT_CENTER_HZ;
            // int MaxFFTIndex = 0;
            // float MaxFFTValue = 0;
            // Find Max :
            // for (size_t i = 2; i < (PrivateData.FFTSampleBox[x].size() / 2); i++)
            // {
            //     if (std::abs(PrivateData.FFTSampleBox[x][i]) > MaxFFTValue)
            //     {
            //         MaxFFTValue = std::abs(PrivateData.FFTSampleBox[x][i]);
            //         MaxFFTIndex = i;
            //     }
            // }

            // centerFreq = freqPeak(PrivateData.FFTSampleBox[x].data(), PrivateData.FFTSampleBox[x].size(), 1, 500);

            // PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[x] = biquadFilterApply(&GryoFilterDynamicFreq[x], centerFreq);
            // PrivateData._uORB_Gyro_Dynamic_NotchCenterHZ[1] = centerFreq;
            // PrivateData.fftindexs[x] = MaxFFTIndex;}
        }
    };

    inline int GetTimestamp()
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        return (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))) - IMUstartuptime);
    }

    std::mutex deviceLock;
    int IMUstartuptime = 0;
    int Sensor_fd;
    float MPU9250_Gryo_LSB = MPU9250_GYRO_LSB;   // +-2000dps
    float MPU9250_Accel_LSB = MPU9250_ACCEL_LSB; //+-16g
    uint8_t Tmp_MPU9250_Buffer[20] = {0};
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
    
    // CleanFlight Filter
    pt1Filter_t VibeLPF[3];
    pt1Filter_t VibeFloorLPF[3];
    pt1Filter_t GryoFilterLPF[3];
    pt1Filter_t GryoFilterLPFST2[3];
    pt1Filter_t AccelFilterLPF[3];
    biquadFilter_t GryoFilterBLPF[3];
    biquadFilter_t GryoFilterBLPFST2[3];
    biquadFilter_t AccelFilterBLPF[3];
    biquadFilter_t GyroNotchFilter[3];
    biquadFilter_t AccelNotchFilter[3];
    // Dynamic Filter
    int FFTCountDown = 0;
    int FFTAsixSampleCount = 0;
    int FFTOverSampleCount = 0;
    int GyroDynamicFFTSampleCount = 0;
    int GyroDynamicFFTCaculateCount = 0;
    bool GyroDynamicNotchReady = false;
    std::vector<float> FFTWINDOW;
    std::vector<std::vector<float>> FFTData;
    float FFTOverSample[3] = {0};
    int GyroDynmiacNotchMinBox = 2;
    int GyroDynamicNotchCenterLast[3] = {350, 350, 350};
    biquadFilter_t GryoFilterDynamicNotch[3];
    biquadFilter_t GryoFilterDynamicFreq[3];
    bool debug_enabled = false;
};