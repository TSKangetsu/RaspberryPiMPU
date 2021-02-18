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

#define MPUALPFAplah 0.985
#define MPUGLPFAplah 0.7
#define MPUMixTraditionAplah 0.9992
#define AccelAvaQ1 30.f
#define AccelAvaQ2 5.f

#define GravityAccel 9.80665
#define MPUGStandard 4096.f

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

    int _uORB_MPU9250_AF_X = 0;
    int _uORB_MPU9250_AF_Y = 0;
    int _uORB_MPU9250_AF_Z = 0;
    int _uORB_MPU9250_ADF_X = 0;
    int _uORB_MPU9250_ADF_Y = 0;
    int _uORB_MPU9250_ADF_Z = 0;

    int _uORB_MPU9250_AStaticFake_X = 0;
    int _uORB_MPU9250_AStaticFake_Y = 0;
    int _uORB_MPU9250_AStaticFake_Z = 0;
    int _uORB_MPU9250_AStaticFakeD_X = 0;
    int _uORB_MPU9250_AStaticFakeD_Y = 0;
    int _uORB_MPU9250_AStaticFakeD_Z = 4096;
    int _uORB_MPU9250_AStaticFakeFD_X = 0;
    int _uORB_MPU9250_AStaticFakeFD_Y = 0;
    int _uORB_MPU9250_AStaticFakeFD_Z = 0;

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

    int _flag_MPU9250_G_X_Cali;
    int _flag_MPU9250_G_Y_Cali;
    int _flag_MPU9250_G_Z_Cali;

    int _flag_MPU9250_A_Static_Raw_X_Cali;
    int _flag_MPU9250_A_Static_Raw_Y_Cali;
    int _flag_MPU9250_A_Static_Raw_Z_Cali;
    int _flag_MPU9250_A_Static_X_Cali;
    int _flag_MPU9250_A_Static_Y_Cali;
    int _flag_MPU9250_A_Static_Z_Cali;

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
                      int MPUSPIChannel = 1, unsigned char MPUI2CAddr = 0x68, int UpdateFreq = 250,
                      int MixFilterType = MPUMixTradition)
    {
        MPU9250_Type = Type;
        MPUUpdateFreq = UpdateFreq;
        MPU9250_I2CAddr = MPU9250_I2CAddr;
        MPU9250_SPI_Channel = MPUSPIChannel;
        MPU9250_MixFilterType = MixFilterType;
        CompassEnable = IsBuildInCompassEnable;

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

        PrivateData._uORB_Gryo_Pitch = (PrivateData._uORB_Gryo_Pitch * MPUGLPFAplah) + ((PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB) * (1.f - MPUGLPFAplah));
        PrivateData._uORB_Gryo__Roll = (PrivateData._uORB_Gryo__Roll * MPUGLPFAplah) + ((PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB) * (1.f - MPUGLPFAplah));
        PrivateData._uORB_Gryo___Yaw = (PrivateData._uORB_Gryo___Yaw * MPUGLPFAplah) + ((PrivateData._uORB_MPU9250_G_Z / MPU9250_Gryo_LSB) * (1.f - MPUGLPFAplah));

        PrivateData._uORB_Real_Pitch += (PrivateData._uORB_MPU9250_G_X / MPU9250_Gryo_LSB) / MPUUpdateFreq;
        PrivateData._uORB_Real__Roll += (PrivateData._uORB_MPU9250_G_Y / MPU9250_Gryo_LSB) / MPUUpdateFreq;
        PrivateData._uORB_Real_Pitch += PrivateData._uORB_Real__Roll * sin((PrivateData._uORB_MPU9250_G_Z / MPUUpdateFreq / MPU9250_Gryo_LSB) * (PI / 180.f));
        PrivateData._uORB_Real__Roll -= PrivateData._uORB_Real_Pitch * sin((PrivateData._uORB_MPU9250_G_Z / MPUUpdateFreq / MPU9250_Gryo_LSB) * (PI / 180.f));

        PrivateData._uORB_MPU9250_AF_X = PrivateData._uORB_MPU9250_AF_X * MPUALPFAplah + PrivateData._uORB_MPU9250_A_X * (1.f - MPUALPFAplah);
        PrivateData._uORB_MPU9250_AF_Y = PrivateData._uORB_MPU9250_AF_Y * MPUALPFAplah + PrivateData._uORB_MPU9250_A_Y * (1.f - MPUALPFAplah);
        PrivateData._uORB_MPU9250_AF_Z = PrivateData._uORB_MPU9250_AF_Z * MPUALPFAplah + PrivateData._uORB_MPU9250_A_Z * (1.f - MPUALPFAplah);
        PrivateData._uORB_MPU9250_AStaticFake_X = sin(PrivateData._uORB_Real__Roll * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector;
        PrivateData._uORB_MPU9250_AStaticFake_Y = sin(PrivateData._uORB_Real_Pitch * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector;
        PrivateData._uORB_MPU9250_AStaticFake_Z = sqrt(PrivateData._uORB_MPU9250_Accel_Static_Vector * PrivateData._uORB_MPU9250_Accel_Static_Vector -
                                                       (sin(PrivateData._uORB_Real__Roll * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector) *
                                                           (sin(PrivateData._uORB_Real__Roll * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector) -
                                                       (sin(PrivateData._uORB_Real_Pitch * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector) *
                                                           (sin(PrivateData._uORB_Real_Pitch * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector));
        PrivateData._uORB_MPU9250_AStaticFakeD_X = PrivateData._uORB_MPU9250_AStaticFakeD_X * MPUALPFAplah + PrivateData._uORB_MPU9250_AStaticFake_X * (1.f - MPUALPFAplah);
        PrivateData._uORB_MPU9250_AStaticFakeD_Y = PrivateData._uORB_MPU9250_AStaticFakeD_Y * MPUALPFAplah + PrivateData._uORB_MPU9250_AStaticFake_Y * (1.f - MPUALPFAplah);
        PrivateData._uORB_MPU9250_AStaticFakeD_Z = PrivateData._uORB_MPU9250_AStaticFakeD_Z * MPUALPFAplah + PrivateData._uORB_MPU9250_AStaticFake_Z * (1.f - MPUALPFAplah);
        {
            //=========================================================================================
            MPU9250_A_AverageFilter_X_Total -= MPU9250_A_AverageFilter_X[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilter_X[MPU9250_AverageFilter_Clock] = PrivateData._uORB_MPU9250_AF_X;
            MPU9250_A_AverageFilter_X_Total += MPU9250_A_AverageFilter_X[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilter_Y_Total -= MPU9250_A_AverageFilter_Y[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilter_Y[MPU9250_AverageFilter_Clock] = PrivateData._uORB_MPU9250_AF_Y;
            MPU9250_A_AverageFilter_Y_Total += MPU9250_A_AverageFilter_Y[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilter_Z_Total -= MPU9250_A_AverageFilter_Z[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilter_Z[MPU9250_AverageFilter_Clock] = PrivateData._uORB_MPU9250_AF_Z;
            MPU9250_A_AverageFilter_Z_Total += MPU9250_A_AverageFilter_Z[MPU9250_AverageFilter_Clock];

            MPU9250_A_AverageFilterA_X_Total -= MPU9250_A_AverageFilterA_X[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilterA_X[MPU9250_AverageFilter_Clock] = PrivateData._uORB_MPU9250_AStaticFakeD_X;
            MPU9250_A_AverageFilterA_X_Total += MPU9250_A_AverageFilterA_X[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilterA_Y_Total -= MPU9250_A_AverageFilterA_Y[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilterA_Y[MPU9250_AverageFilter_Clock] = PrivateData._uORB_MPU9250_AStaticFakeD_Y;
            MPU9250_A_AverageFilterA_Y_Total += MPU9250_A_AverageFilterA_Y[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilterA_Z_Total -= MPU9250_A_AverageFilterA_Z[MPU9250_AverageFilter_Clock];
            MPU9250_A_AverageFilterA_Z[MPU9250_AverageFilter_Clock] = PrivateData._uORB_MPU9250_AStaticFakeD_Z;
            MPU9250_A_AverageFilterA_Z_Total += MPU9250_A_AverageFilterA_Z[MPU9250_AverageFilter_Clock];
            //=========================================================================================
            MPU9250_AverageFilter_Clock++;
            if (MPU9250_AverageFilter_Clock == (int)AccelAvaQ1)
            {
                //=========================================================================================
                MPU9250_A_AverageFilterD_X_Total -= MPU9250_A_AverageFilterD_X[MPU9250_AverageFilterD_Clock];
                MPU9250_A_AverageFilterD_X[MPU9250_AverageFilterD_Clock] = MPU9250_A_AverageFilter_X_Total / AccelAvaQ1;
                MPU9250_A_AverageFilterD_X_Total += MPU9250_A_AverageFilterD_X[MPU9250_AverageFilterD_Clock];
                PrivateData._uORB_MPU9250_ADF_X = MPU9250_A_AverageFilterD_X_Total / AccelAvaQ2;
                MPU9250_A_AverageFilterD_Y_Total -= MPU9250_A_AverageFilterD_Y[MPU9250_AverageFilterD_Clock];
                MPU9250_A_AverageFilterD_Y[MPU9250_AverageFilterD_Clock] = MPU9250_A_AverageFilter_Y_Total / AccelAvaQ1;
                MPU9250_A_AverageFilterD_Y_Total += MPU9250_A_AverageFilterD_Y[MPU9250_AverageFilterD_Clock];
                PrivateData._uORB_MPU9250_ADF_Y = MPU9250_A_AverageFilterD_Y_Total / AccelAvaQ2;
                MPU9250_A_AverageFilterD_Z_Total -= MPU9250_A_AverageFilterD_Z[MPU9250_AverageFilterD_Clock];
                MPU9250_A_AverageFilterD_Z[MPU9250_AverageFilterD_Clock] = MPU9250_A_AverageFilter_Z_Total / AccelAvaQ1;
                MPU9250_A_AverageFilterD_Z_Total += MPU9250_A_AverageFilterD_Z[MPU9250_AverageFilterD_Clock];
                PrivateData._uORB_MPU9250_ADF_Z = MPU9250_A_AverageFilterD_Z_Total / AccelAvaQ2;

                MPU9250_A_AverageFilterAD_X_Total -= MPU9250_A_AverageFilterAD_X[MPU9250_AverageFilterD_Clock];
                MPU9250_A_AverageFilterAD_X[MPU9250_AverageFilterD_Clock] = MPU9250_A_AverageFilterA_X_Total / AccelAvaQ1;
                MPU9250_A_AverageFilterAD_X_Total += MPU9250_A_AverageFilterAD_X[MPU9250_AverageFilterD_Clock];
                PrivateData._uORB_MPU9250_AStaticFakeFD_X = MPU9250_A_AverageFilterAD_X_Total / AccelAvaQ2;
                MPU9250_A_AverageFilterAD_Y_Total -= MPU9250_A_AverageFilterAD_Y[MPU9250_AverageFilterD_Clock];
                MPU9250_A_AverageFilterAD_Y[MPU9250_AverageFilterD_Clock] = MPU9250_A_AverageFilterA_Y_Total / AccelAvaQ1;
                MPU9250_A_AverageFilterAD_Y_Total += MPU9250_A_AverageFilterAD_Y[MPU9250_AverageFilterD_Clock];
                PrivateData._uORB_MPU9250_AStaticFakeFD_Y = MPU9250_A_AverageFilterAD_Y_Total / AccelAvaQ2;
                MPU9250_A_AverageFilterAD_Z_Total -= MPU9250_A_AverageFilterAD_Z[MPU9250_AverageFilterD_Clock];
                MPU9250_A_AverageFilterAD_Z[MPU9250_AverageFilterD_Clock] = MPU9250_A_AverageFilterA_Z_Total / AccelAvaQ1;
                MPU9250_A_AverageFilterAD_Z_Total += MPU9250_A_AverageFilterAD_Z[MPU9250_AverageFilterD_Clock];
                PrivateData._uORB_MPU9250_AStaticFakeFD_Z = MPU9250_A_AverageFilterAD_Z_Total / AccelAvaQ2;
                //=========================================================================================
                MPU9250_AverageFilterD_Clock++;
                if (MPU9250_AverageFilterD_Clock == (int)AccelAvaQ2)
                    MPU9250_AverageFilterD_Clock = 0;

                MPU9250_AverageFilter_Clock = 0;
            }
            //=========================================================================================
        }
        PrivateData._uORB_MPU9250_A_Vector = sqrt((PrivateData._uORB_MPU9250_ADF_X * PrivateData._uORB_MPU9250_ADF_X) +
                                                  (PrivateData._uORB_MPU9250_ADF_Y * PrivateData._uORB_MPU9250_ADF_Y) +
                                                  (PrivateData._uORB_MPU9250_ADF_Z * PrivateData._uORB_MPU9250_ADF_Z));
        PrivateData._uORB_Accel_Pitch = atan2((float)PrivateData._uORB_MPU9250_ADF_Y, PrivateData._uORB_MPU9250_ADF_Z) * 180 / PI;
        PrivateData._uORB_Accel__Roll = atan2((float)PrivateData._uORB_MPU9250_ADF_X, PrivateData._uORB_MPU9250_ADF_Z) * 180 / PI;
        PrivateData._uORB_MPU9250_A_Static_Raw_X = PrivateData._uORB_MPU9250_AStaticFakeFD_X - PrivateData._uORB_MPU9250_ADF_X;
        PrivateData._uORB_MPU9250_A_Static_Raw_Y = PrivateData._uORB_MPU9250_AStaticFakeFD_Y - PrivateData._uORB_MPU9250_ADF_Y;
        PrivateData._uORB_MPU9250_A_Static_Raw_Z = PrivateData._uORB_MPU9250_AStaticFakeFD_Z - PrivateData._uORB_MPU9250_ADF_Z;
        //=========================
        PrivateData._uORB_MPU9250_A_Static_X = PrivateData._uORB_MPU9250_A_Static_Raw_X * cos(-1 * PrivateData._uORB_Real__Roll * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_X += PrivateData._uORB_MPU9250_A_Static_Raw_Z * sin(-1 * PrivateData._uORB_Real__Roll * (PI / 180.f));

        PrivateData._uORB_MPU9250_A_Static_Y = PrivateData._uORB_MPU9250_A_Static_Raw_Y * cos(-1 * PrivateData._uORB_Real_Pitch * (PI / 180.f));
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
    int MPUUpdateFreq = 250;
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

    double _uORB_MPU9250_A_Fake_Static_X = 0;
    double _uORB_MPU9250_A_Fake_Static_Y = 0;
    double _uORB_MPU9250_A_Fake_Static_Z = 0;

    int MPU9250_AverageFilter_Clock = 0;
    int MPU9250_A_AverageFilter_X[50] = {0};
    int MPU9250_A_AverageFilter_Y[50] = {0};
    int MPU9250_A_AverageFilter_Z[50] = {0};
    double MPU9250_A_AverageFilter_X_Total = 0;
    double MPU9250_A_AverageFilter_Y_Total = 0;
    double MPU9250_A_AverageFilter_Z_Total = 0;

    int MPU9250_AverageFilterD_Clock = 0;
    int MPU9250_A_AverageFilterD_X[100] = {0};
    int MPU9250_A_AverageFilterD_Y[100] = {0};
    int MPU9250_A_AverageFilterD_Z[100] = {0};
    double MPU9250_A_AverageFilterD_X_Total = 0;
    double MPU9250_A_AverageFilterD_Y_Total = 0;
    double MPU9250_A_AverageFilterD_Z_Total = 0;

    int MPU9250_AverageFilterA_Clock = 0;
    int MPU9250_A_AverageFilterA_X[50] = {0};
    int MPU9250_A_AverageFilterA_Y[50] = {0};
    int MPU9250_A_AverageFilterA_Z[50] = {0};
    double MPU9250_A_AverageFilterA_X_Total = 0;
    double MPU9250_A_AverageFilterA_Y_Total = 0;
    double MPU9250_A_AverageFilterA_Z_Total = 0;

    int MPU9250_AverageFilterAD_Clock = 0;
    int MPU9250_A_AverageFilterAD_X[100] = {0};
    int MPU9250_A_AverageFilterAD_Y[100] = {0};
    int MPU9250_A_AverageFilterAD_Z[100] = {0};
    double MPU9250_A_AverageFilterAD_X_Total = 0;
    double MPU9250_A_AverageFilterAD_Y_Total = 0;
    double MPU9250_A_AverageFilterAD_Z_Total = 0;
};