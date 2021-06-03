#include "filter.h"
#include <math.h>
#include <thread>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>

#define PI 3.1415926
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
#define MPUGStandard 4096.f

#define DYNAMIC_NOTCH_DEFAULT_CENTER_HZ 350.f
#define ACC_VIBE_FLOOR_FILT_HZ 5.f
#define ACC_VIBE_FILT_HZ 2.f

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

    float _uORB_MPU9250_AStaticFake_X = 0;
    float _uORB_MPU9250_AStaticFake_Y = 0;
    float _uORB_MPU9250_AStaticFake_Z = 0;
    float _uORB_MPU9250_AStaticFakeFD_X = 0;
    float _uORB_MPU9250_AStaticFakeFD_Y = 0;
    float _uORB_MPU9250_AStaticFakeFD_Z = 0;

    int _uORB_MPU9250_A_Vector = 0;
    int _uORB_MPU9250_Accel_Static_Vector = 4250;
    int _uORB_MPU9250_A_Static_X = 0;
    int _uORB_MPU9250_A_Static_Y = 0;
    int _uORB_MPU9250_A_Static_Z = 0;
    int _uORB_MPU9250_A_Static_Raw_X = 0;
    int _uORB_MPU9250_A_Static_Raw_Y = 0;
    int _uORB_MPU9250_A_Static_Raw_Z = 0;

    float _uORB_Gryo__Roll = 0;
    float _uORB_Gryo_Pitch = 0;
    float _uORB_Gryo___Yaw = 0;
    float _uORB_Real__Roll = 0;
    float _uORB_Real_Pitch = 0;
    float _uORB_Accel__Roll = 0;
    float _uORB_Accel_Pitch = 0;
    float _uORB_Acceleration_X = 0;
    float _uORB_Acceleration_Y = 0;
    float _uORB_Acceleration_Z = 0;
    bool _uORB_Real_Reverse = false;

    float _uORB_Accel_VIBE_X = 0;
    float _uORB_Accel_VIBE_Y = 0;
    float _uORB_Accel_VIBE_Z = 0;

    int _flag_MPU9250_G_X_Cali;
    int _flag_MPU9250_G_Y_Cali;
    int _flag_MPU9250_G_Z_Cali;

    int _flag_MPU9250_A_Static_X_Cali;
    int _flag_MPU9250_A_Static_Y_Cali;
    int _flag_MPU9250_A_Static_Z_Cali;
    int _flag_MPU9250_A_Static_Raw_X_Cali;
    int _flag_MPU9250_A_Static_Raw_Y_Cali;
    int _flag_MPU9250_A_Static_Raw_Z_Cali;

    double _flag_MPU9250_A_X_Scal;
    double _flag_MPU9250_A_Y_Scal;
    double _flag_MPU9250_A_Z_Scal;
    double _flag_MPU9250_A_X_Cali;
    double _flag_MPU9250_A_Y_Cali;
    double _flag_MPU9250_A_Z_Cali;
};

class RPiMPU9250
{
public:
    inline RPiMPU9250(int Type = MPUTypeSPI, bool IsBuildInCompassEnable = false,
                      int MPUSPIChannel = 1, unsigned char MPUI2CAddr = 0x68, int UpdateFreq = 1000,
                      int MixFilterType = MPUMixTradition, float MPUMixAplah = 0.9996,
                      int GFilterType = FilterLPFBiquad, int GCutOff = 256,
                      int AccFilterType = FilterLPFBiquad, int AccCutOff = 15)
    {
        MPU9250_Type = Type;
        MPUUpdateFreq = UpdateFreq;
        MPU9250_I2CAddr = MPUI2CAddr;
        MPU9250_SPI_Channel = MPUSPIChannel;
        MPU9250_MixFilterType = MixFilterType;
        CompassEnable = IsBuildInCompassEnable;

        GryoFilterType = GFilterType;
        AccelFilterType = AccFilterType;
        MPUMixTraditionAplah = MPUMixAplah;

        int DT = (float)(1.f / (float)UpdateFreq) * 1000000;
        switch (GryoFilterType)
        {
        case FilterLPFPT1:
            pt1FilterInit(&GryoFilterLPFX, GCutOff, DT * 1e-6f);
            pt1FilterInit(&GryoFilterLPFY, GCutOff, DT * 1e-6f);
            pt1FilterInit(&GryoFilterLPFZ, GCutOff, DT * 1e-6f);
            break;

        case FilterLPFBiquad:
            biquadFilterInitLPF(&GryoFilterBLPFX, GCutOff, DT);
            biquadFilterInitLPF(&GryoFilterBLPFY, GCutOff, DT);
            biquadFilterInitLPF(&GryoFilterBLPFZ, GCutOff, DT);
            break;
        }

        biquadFilterInit(&GryoFilterNotchX, DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
        biquadFilterInit(&GryoFilterNotchY, DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
        biquadFilterInit(&GryoFilterNotchZ, DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);

        switch (AccelFilterType)
        {
        case FilterLPFPT1:
            pt1FilterInit(&AccelFilterLPFX, AccCutOff, DT * 1e-6f);
            pt1FilterInit(&AccelFilterLPFY, AccCutOff, DT * 1e-6f);
            pt1FilterInit(&AccelFilterLPFZ, AccCutOff, DT * 1e-6f);
            pt1FilterInit(&FakeAccelFilterLPFX, AccCutOff, DT * 1e-6f);
            pt1FilterInit(&FakeAccelFilterLPFY, AccCutOff, DT * 1e-6f);
            pt1FilterInit(&FakeAccelFilterLPFZ, AccCutOff, DT * 1e-6f);
            break;

        case FilterLPFBiquad:
            biquadFilterInitLPF(&AccelFilterBLPFX, AccCutOff, DT);
            biquadFilterInitLPF(&AccelFilterBLPFY, AccCutOff, DT);
            biquadFilterInitLPF(&AccelFilterBLPFZ, AccCutOff, DT);
            biquadFilterInitLPF(&FakeAccelFilterBLPFX, AccCutOff, DT);
            biquadFilterInitLPF(&FakeAccelFilterBLPFY, AccCutOff, DT);
            biquadFilterInitLPF(&FakeAccelFilterBLPFZ, AccCutOff, DT);
            break;
        }

        pt1FilterInit(&VibeFloorLPFX, ACC_VIBE_FLOOR_FILT_HZ, DT * 1e-6f);
        pt1FilterInit(&VibeFloorLPFY, ACC_VIBE_FLOOR_FILT_HZ, DT * 1e-6f);
        pt1FilterInit(&VibeFloorLPFZ, ACC_VIBE_FLOOR_FILT_HZ, DT * 1e-6f);
        pt1FilterInit(&VibeLPFX, ACC_VIBE_FILT_HZ, DT * 1e-6f);
        pt1FilterInit(&VibeLPFY, ACC_VIBE_FILT_HZ, DT * 1e-6f);
        pt1FilterInit(&VibeLPFZ, ACC_VIBE_FILT_HZ, DT * 1e-6f);

        if (Type == MPUTypeSPI)
        {
            MPU9250_fd = wiringPiSPISetup(MPU9250_SPI_Channel, MPU9250_SPI_Freq);
            MPU9250_SPI_Config[0] = 0x6b;
            MPU9250_SPI_Config[1] = 0x00;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, MPU9250_SPI_Config, 2); //reset
            MPU9250_SPI_Config[0] = 0x1c;
            MPU9250_SPI_Config[1] = 0x10;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, MPU9250_SPI_Config, 2); // Accel
            MPU9250_SPI_Config[0] = 0x1b;
            MPU9250_SPI_Config[1] = 0x08;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, MPU9250_SPI_Config, 2); // Gryo
            MPU9250_SPI_Config[0] = 0x1a;
            MPU9250_SPI_Config[1] = 0x03;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, MPU9250_SPI_Config, 2); //config
            if (CompassEnable)
            {
            }
        }
        else if (Type == MPUTypeI2C)
        {
            MPU9250_fd = wiringPiI2CSetup(MPU9250_I2CAddr);
            wiringPiI2CWriteReg8(MPU9250_fd, 107, 0x00); //reset
            wiringPiI2CWriteReg8(MPU9250_fd, 28, 0x10);  //Accel
            wiringPiI2CWriteReg8(MPU9250_fd, 27, 0x08);  //Gryo
            wiringPiI2CWriteReg8(MPU9250_fd, 26, 0x03);  //config
            if (CompassEnable)
            {
            }
        }
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
        double _Tmp_Static_Raw_X_Cali = 0;
        double _Tmp_Static_Raw_Y_Cali = 0;
        double _Tmp_Static_Raw_Z_Cali = 0;
        double _Tmp_Static_X_Cali = 0;
        double _Tmp_Static_Y_Cali = 0;
        double _Tmp_Static_Z_Cali = 0;
        double _Tmp_Static_Vector = 0;

        PrivateData._flag_MPU9250_G_X_Cali = 0;
        PrivateData._flag_MPU9250_G_Y_Cali = 0;
        PrivateData._flag_MPU9250_G_Z_Cali = 0;
        PrivateData._flag_MPU9250_A_Static_Raw_X_Cali = 0;
        PrivateData._flag_MPU9250_A_Static_Raw_Y_Cali = 0;
        PrivateData._flag_MPU9250_A_Static_Raw_Z_Cali = 0;
        PrivateData._flag_MPU9250_A_Static_X_Cali = 0;
        PrivateData._flag_MPU9250_A_Static_Y_Cali = 0;
        PrivateData._flag_MPU9250_A_Static_Z_Cali = 0;
        PrivateData._uORB_MPU9250_Accel_Static_Vector = 4096;
        for (size_t i = 0; i < 2000; i++)
        {
            MPUSensorsDataGet();
            ResetMPUMixAngle();
            PrivateData._uORB_MPU9250_A_Vector = sqrt((PrivateData._uORB_MPU9250_A_X * PrivateData._uORB_MPU9250_A_X) +
                                                      (PrivateData._uORB_MPU9250_A_Y * PrivateData._uORB_MPU9250_A_Y) +
                                                      (PrivateData._uORB_MPU9250_A_Z * PrivateData._uORB_MPU9250_A_Z));
            _Tmp_Static_Vector += PrivateData._uORB_MPU9250_A_Vector;
            usleep((int)(1.f / (float)MPUUpdateFreq * 1000000.f));
        }
        PrivateData._uORB_MPU9250_Accel_Static_Vector = _Tmp_Static_Vector / 2000.f;
        for (int cali_count = 0; cali_count < 1000; cali_count++)
        {
            MPUSensorsDataGet();
            ResetMPUMixAngle();
            _Tmp_Gryo_X_Cali += PrivateData._uORB_MPU9250_G_X;
            _Tmp_Gryo_Y_Cali += PrivateData._uORB_MPU9250_G_Y;
            _Tmp_Gryo_Z_Cali += PrivateData._uORB_MPU9250_G_Z;
            _Tmp_Static_Raw_X_Cali += PrivateData._uORB_MPU9250_A_Static_Raw_X;
            _Tmp_Static_Raw_Y_Cali += PrivateData._uORB_MPU9250_A_Static_Raw_Y;
            _Tmp_Static_Raw_Z_Cali += PrivateData._uORB_MPU9250_A_Static_Raw_Z;
            usleep((int)(1.f / (float)MPUUpdateFreq * 1000000.f));
        }
        PrivateData._flag_MPU9250_G_X_Cali = _Tmp_Gryo_X_Cali / 1000.0;
        PrivateData._flag_MPU9250_G_Y_Cali = _Tmp_Gryo_Y_Cali / 1000.0;
        PrivateData._flag_MPU9250_G_Z_Cali = _Tmp_Gryo_Z_Cali / 1000.0;
        PrivateData._flag_MPU9250_A_Static_Raw_X_Cali = _Tmp_Static_Raw_X_Cali / 1000.0;
        PrivateData._flag_MPU9250_A_Static_Raw_Y_Cali = _Tmp_Static_Raw_Y_Cali / 1000.0;
        PrivateData._flag_MPU9250_A_Static_Raw_Z_Cali = _Tmp_Static_Raw_Z_Cali / 1000.0;

        for (int cali_count = 0; cali_count < 1000; cali_count++)
        {
            MPUSensorsDataGet();
            ResetMPUMixAngle();
            _Tmp_Static_X_Cali += PrivateData._uORB_MPU9250_A_Static_X;
            _Tmp_Static_Y_Cali += PrivateData._uORB_MPU9250_A_Static_Y;
            _Tmp_Static_Z_Cali += PrivateData._uORB_MPU9250_A_Static_Z;
            usleep((int)(1.f / (float)MPUUpdateFreq * 1000000.f));
        }
        PrivateData._flag_MPU9250_A_Static_X_Cali = _Tmp_Static_X_Cali / 1000.0;
        PrivateData._flag_MPU9250_A_Static_Y_Cali = _Tmp_Static_Y_Cali / 1000.0;
        PrivateData._flag_MPU9250_A_Static_Z_Cali = _Tmp_Static_Z_Cali / 1000.0;
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
            usleep((int)(1.f / (float)MPUUpdateFreq * 1000000.f));
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
            AccelCaliData[MPUAccelScalX] = MPUGStandard / (AccelCaliData[MPUAccelNoseLeft] - AccelCaliData[7]);
            AccelCaliData[MPUAccelScalY] = MPUGStandard / (AccelCaliData[MPUAccelNoseUp] - AccelCaliData[8]);
            AccelCaliData[MPUAccelScalZ] = MPUGStandard / (AccelCaliData[MPUAccelNoseTop] - AccelCaliData[9]);
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

        switch (GryoFilterType)
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

        if (PrivateData._uORB_Real__Roll > 90.f)
        {
            GryoReveresRoll *= -1;
            PrivateData._uORB_Real_Reverse = !PrivateData._uORB_Real_Reverse;
        }
        if (PrivateData._uORB_Real__Roll < -90.f)
        {
            GryoReveresRoll *= -1;
            PrivateData._uORB_Real_Reverse = !PrivateData._uORB_Real_Reverse;
        }
        if (PrivateData._uORB_Real_Pitch > 90.f)
        {
            GryoReveresPitch *= -1;
            PrivateData._uORB_Real_Reverse = !PrivateData._uORB_Real_Reverse;
        }
        if (PrivateData._uORB_Real_Pitch < -90.f)
        {
            GryoReveresPitch *= -1;
            PrivateData._uORB_Real_Reverse = !PrivateData._uORB_Real_Reverse;
        }

        PrivateData._uORB_Real__Roll += GryoReveresRoll * ((PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB) / MPUUpdateFreq);
        PrivateData._uORB_Real_Pitch += GryoReveresPitch * ((PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB) / MPUUpdateFreq);

        PrivateData._uORB_Real_Pitch += PrivateData._uORB_Real__Roll * sin((PrivateData._uORB_MPU9250_G_Z / MPUUpdateFreq / MPU9250_Gryo_LSB) * (PI / 180.f));
        PrivateData._uORB_Real__Roll -= PrivateData._uORB_Real_Pitch * sin((PrivateData._uORB_MPU9250_G_Z / MPUUpdateFreq / MPU9250_Gryo_LSB) * (PI / 180.f));

        PrivateData._uORB_MPU9250_AStaticFake_X = sin(PrivateData._uORB_Real__Roll * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector;
        PrivateData._uORB_MPU9250_AStaticFake_Y = sin(PrivateData._uORB_Real_Pitch * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector;
        PrivateData._uORB_MPU9250_AStaticFake_Z =
            (float)sqrt(abs(pow(PrivateData._uORB_MPU9250_Accel_Static_Vector, 2) -
                            pow(PrivateData._uORB_MPU9250_AStaticFake_X, 2) -
                            pow(PrivateData._uORB_MPU9250_AStaticFake_Y, 2)));
        //========================= //=========================
        switch (AccelFilterType)
        {
        case FilterLPFPT1:
            PrivateData._uORB_MPU9250_ADF_X = pt1FilterApply(&AccelFilterLPFX, (float)PrivateData._uORB_MPU9250_A_X);
            PrivateData._uORB_MPU9250_ADF_Y = pt1FilterApply(&AccelFilterLPFY, (float)PrivateData._uORB_MPU9250_A_Y);
            PrivateData._uORB_MPU9250_ADF_Z = pt1FilterApply(&AccelFilterLPFZ, (float)PrivateData._uORB_MPU9250_A_Z);
            PrivateData._uORB_MPU9250_AStaticFakeFD_X = pt1FilterApply(&FakeAccelFilterLPFX, (float)PrivateData._uORB_MPU9250_AStaticFake_X);
            PrivateData._uORB_MPU9250_AStaticFakeFD_Y = pt1FilterApply(&FakeAccelFilterLPFY, (float)PrivateData._uORB_MPU9250_AStaticFake_Y);
            PrivateData._uORB_MPU9250_AStaticFakeFD_Z = pt1FilterApply(&FakeAccelFilterLPFZ, (float)PrivateData._uORB_MPU9250_AStaticFake_Z);
            break;
        case FilterLPFBiquad:
            PrivateData._uORB_MPU9250_ADF_X = biquadFilterApply(&AccelFilterBLPFX, (float)PrivateData._uORB_MPU9250_A_X);
            PrivateData._uORB_MPU9250_ADF_Y = biquadFilterApply(&AccelFilterBLPFY, (float)PrivateData._uORB_MPU9250_A_Y);
            PrivateData._uORB_MPU9250_ADF_Z = biquadFilterApply(&AccelFilterBLPFZ, (float)PrivateData._uORB_MPU9250_A_Z);
            PrivateData._uORB_MPU9250_AStaticFakeFD_X = biquadFilterApply(&FakeAccelFilterBLPFX, (float)PrivateData._uORB_MPU9250_AStaticFake_X);
            PrivateData._uORB_MPU9250_AStaticFakeFD_Y = biquadFilterApply(&FakeAccelFilterBLPFY, (float)PrivateData._uORB_MPU9250_AStaticFake_Y);
            PrivateData._uORB_MPU9250_AStaticFakeFD_Z = biquadFilterApply(&FakeAccelFilterBLPFZ, (float)PrivateData._uORB_MPU9250_AStaticFake_Z);
            break;
        }
        if (PrivateData._uORB_Real_Reverse)
            PrivateData._uORB_MPU9250_AStaticFakeFD_Z *= -1;
        //========================= //=========================
        PrivateData._uORB_MPU9250_A_Vector = sqrt((PrivateData._uORB_MPU9250_ADF_X * PrivateData._uORB_MPU9250_ADF_X) +
                                                  (PrivateData._uORB_MPU9250_ADF_Y * PrivateData._uORB_MPU9250_ADF_Y) +
                                                  (PrivateData._uORB_MPU9250_ADF_Z * PrivateData._uORB_MPU9250_ADF_Z));

        PrivateData._uORB_Accel_Pitch = atan2((float)PrivateData._uORB_MPU9250_ADF_Y, PrivateData._uORB_MPU9250_ADF_Z) * 180.f / PI;
        PrivateData._uORB_Accel__Roll = atan2((float)PrivateData._uORB_MPU9250_ADF_X, PrivateData._uORB_MPU9250_ADF_Z) * 180.f / PI;
        if (PrivateData._uORB_Accel_Pitch > 90.f)
            PrivateData._uORB_Accel_Pitch = 180.f - PrivateData._uORB_Accel_Pitch;
        else if (PrivateData._uORB_Accel_Pitch < -90.f)
            PrivateData._uORB_Accel_Pitch = -1 * (180.f - (-1 * PrivateData._uORB_Accel_Pitch));

        if (PrivateData._uORB_Accel__Roll > 90.f)
            PrivateData._uORB_Accel__Roll = 180.f - PrivateData._uORB_Accel__Roll;
        else if (PrivateData._uORB_Accel__Roll < -90.f)
            PrivateData._uORB_Accel__Roll = -1 * (180.f - (-1 * PrivateData._uORB_Accel__Roll));

        PrivateData._uORB_MPU9250_A_Static_Raw_X = PrivateData._uORB_MPU9250_AStaticFakeFD_X - PrivateData._uORB_MPU9250_ADF_X;
        PrivateData._uORB_MPU9250_A_Static_Raw_Y = PrivateData._uORB_MPU9250_AStaticFakeFD_Y - PrivateData._uORB_MPU9250_ADF_Y;
        PrivateData._uORB_MPU9250_A_Static_Raw_Z = PrivateData._uORB_MPU9250_AStaticFakeFD_Z - PrivateData._uORB_MPU9250_ADF_Z;
        //=========================
        PrivateData._uORB_MPU9250_A_Static_X = PrivateData._uORB_MPU9250_A_Static_Raw_X * cos(-1 * PrivateData._uORB_Real__Roll * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_X += PrivateData._uORB_MPU9250_A_Static_Raw_Y * sin(-1 * PrivateData._uORB_Real_Pitch * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_X += PrivateData._uORB_MPU9250_A_Static_Raw_Z * sin(-1 * PrivateData._uORB_Real__Roll * (PI / 180.f));

        PrivateData._uORB_MPU9250_A_Static_Y = PrivateData._uORB_MPU9250_A_Static_Raw_Y * cos(-1 * PrivateData._uORB_Real_Pitch * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_Y += PrivateData._uORB_MPU9250_A_Static_Raw_X * sin(-1 * PrivateData._uORB_Real__Roll * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_Y += PrivateData._uORB_MPU9250_A_Static_Raw_Z * sin(-1 * PrivateData._uORB_Real_Pitch * (PI / 180.f));
        //=========================
        PrivateData._uORB_MPU9250_A_Static_Z = 0;
        PrivateData._uORB_MPU9250_A_Static_Z += PrivateData._uORB_MPU9250_A_Static_Raw_X * sin(-1 * PrivateData._uORB_Real__Roll * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_Z += PrivateData._uORB_MPU9250_A_Static_Raw_Y * sin(-1 * PrivateData._uORB_Real_Pitch * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_Z -= (PrivateData._uORB_MPU9250_A_Static_Raw_Z * cos(-1 * PrivateData._uORB_Real_Pitch * (PI / 180.f)) +
                                                 PrivateData._uORB_MPU9250_A_Static_Raw_Z * cos(-1 * PrivateData._uORB_Real__Roll * (PI / 180.f))) /
                                                2.f;
        //=========================
        PrivateData._uORB_MPU9250_A_Static_Raw_X -= PrivateData._flag_MPU9250_A_Static_Raw_X_Cali;
        PrivateData._uORB_MPU9250_A_Static_Raw_Y -= PrivateData._flag_MPU9250_A_Static_Raw_Y_Cali;
        PrivateData._uORB_MPU9250_A_Static_Raw_Z -= PrivateData._flag_MPU9250_A_Static_Raw_Z_Cali;
        PrivateData._uORB_MPU9250_A_Static_X -= PrivateData._flag_MPU9250_A_Static_X_Cali;
        PrivateData._uORB_MPU9250_A_Static_Y -= PrivateData._flag_MPU9250_A_Static_Y_Cali;
        PrivateData._uORB_MPU9250_A_Static_Z -= PrivateData._flag_MPU9250_A_Static_Z_Cali;

        PrivateData._uORB_Acceleration_X = ((float)PrivateData._uORB_MPU9250_A_Static_X / MPU9250_Accel_LSB) * GravityAccel * 100.f;
        PrivateData._uORB_Acceleration_Y = ((float)PrivateData._uORB_MPU9250_A_Static_Y / MPU9250_Accel_LSB) * GravityAccel * 100.f;
        PrivateData._uORB_Acceleration_Z = ((float)PrivateData._uORB_MPU9250_A_Static_Z / MPU9250_Accel_LSB) * GravityAccel * 100.f;

        if (MPU9250_MixFilterType == MPUMixTradition)
        {
            PrivateData._uORB_Real__Roll = PrivateData._uORB_Real__Roll * MPUMixTraditionAplah + PrivateData._uORB_Accel__Roll * (1.f - MPUMixTraditionAplah);
            PrivateData._uORB_Real_Pitch = PrivateData._uORB_Real_Pitch * MPUMixTraditionAplah + PrivateData._uORB_Accel_Pitch * (1.f - MPUMixTraditionAplah);
        }
        else if (MPU9250_MixFilterType == MPUMixKalman)
        {
            //
            //
        }

        return PrivateData;
    }

    // This function set Total Angle to Accel Angle immediately , Require MPUSensorsDataGet() finish
    inline void ResetMPUMixAngle()
    {
        PrivateData._uORB_Real__Roll = PrivateData._uORB_Accel__Roll;
        PrivateData._uORB_Real_Pitch = PrivateData._uORB_Accel_Pitch;
        if (PrivateData._uORB_MPU9250_ADF_Z > 0)
            PrivateData._uORB_Real_Reverse = false;
        else
            PrivateData._uORB_Real_Reverse = true;
    }

private:
    inline void IMUSensorsDataRead()
    {
        if (MPU9250_Type == MPUTypeI2C)
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
        else if (MPU9250_Type == MPUTypeSPI)
        {
            Tmp_MPU9250_SPI_Buffer[0] = 0xBB;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, Tmp_MPU9250_SPI_Buffer, 21);
            PrivateData._uORB_MPU9250_A_X = (short)((int)Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)Tmp_MPU9250_SPI_Buffer[2]) * -1;
            ;
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

    int MPU9250_fd;
    int MPUUpdateFreq = 1000;
    float MPU9250_Gryo_LSB = 65.5;
    float MPU9250_Accel_LSB = 4096.f;
    bool CompassEnable = false;
    int MPU9250_I2CAddr = 0x68;
    int MPU9250_SPI_Channel = 1;
    int MPU9250_Type = MPUTypeSPI;
    int MPU9250_SPI_Freq = 10000000;
    int MPU9250_MixFilterType = MPUMixTradition;
    unsigned char MPU9250_SPI_Config[20] = {0};
    unsigned char Tmp_MPU9250_Buffer[20] = {0};
    unsigned char Tmp_MPU9250_SPI_Buffer[20] = {0};
    MPUData PrivateData;

    int GryoReveresPitch = 1;
    int GryoReveresRoll = 1;

    double _uORB_MPU9250_A_Fake_Static_X = 0;
    double _uORB_MPU9250_A_Fake_Static_Y = 0;
    double _uORB_MPU9250_A_Fake_Static_Z = 0;

    int GryoFilterType;
    int AccelFilterType;
    float MPUMixTraditionAplah;
    //CleanFlight Filter
    pt1Filter_t GryoFilterLPFX;
    pt1Filter_t GryoFilterLPFY;
    pt1Filter_t GryoFilterLPFZ;

    biquadFilter_t GryoFilterBLPFX;
    biquadFilter_t GryoFilterBLPFY;
    biquadFilter_t GryoFilterBLPFZ;

    biquadFilter_t GryoFilterNotchX;
    biquadFilter_t GryoFilterNotchY;
    biquadFilter_t GryoFilterNotchZ;

    pt1Filter_t AccelFilterLPFX;
    pt1Filter_t AccelFilterLPFY;
    pt1Filter_t AccelFilterLPFZ;
    pt1Filter_t FakeAccelFilterLPFX;
    pt1Filter_t FakeAccelFilterLPFY;
    pt1Filter_t FakeAccelFilterLPFZ;

    biquadFilter_t AccelFilterBLPFX;
    biquadFilter_t AccelFilterBLPFY;
    biquadFilter_t AccelFilterBLPFZ;
    biquadFilter_t FakeAccelFilterBLPFX;
    biquadFilter_t FakeAccelFilterBLPFY;
    biquadFilter_t FakeAccelFilterBLPFZ;

    pt1Filter_t VibeFloorLPFX;
    pt1Filter_t VibeFloorLPFY;
    pt1Filter_t VibeFloorLPFZ;
    pt1Filter_t VibeLPFX;
    pt1Filter_t VibeLPFY;
    pt1Filter_t VibeLPFZ;
};