#include <math.h>
#include <thread>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#define PI 3.1415926
#define MPUTypeI2C 0
#define MPUTypeSPI 1

#define MPUALPFAplah 0.2
#define MPUGLPFAplah 0.7

#define MPUMixKalman 1
#define MPUMixTradition 0

#define MPUMixTraditionAplah 0.9992

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
    int _uORB_MPU9250_AFQ_X = 0;
    int _uORB_MPU9250_AFQ_Y = 0;
    int _uORB_MPU9250_AFQ_Z = 0;
    int _uORB_MPU9250_AFQ2_X = 0;
    int _uORB_MPU9250_AFQ2_Y = 0;
    int _uORB_MPU9250_AFQ2_Z = 0;

    double _uORB_MPU9250_A_Vector = 0;
    double _uORB_MPU9250_A_Static_X = 0;
    double _uORB_MPU9250_A_Static_Y = 0;
    double _uORB_MPU9250_A_Static_Z = 0;

    double _uORB_MPU9250_Accel_To_Static_X = 0;
    double _uORB_MPU9250_Accel_To_Static_Y = 0;
    double _uORB_MPU9250_Accel_To_Static_Z = 0;
    double _uORB_MPU9250_Accel_Static_Vector = 4250;
    double _uORB_MPU9250_Accel_Static_Angle_X = 0;
    double _uORB_MPU9250_Accel_Static_Angle_Y = 0;
    double _uORB_MPU9250_Accel_Static_X_Vector = 0;
    double _uORB_MPU9250_Accel_Static_Y_Vector = 0;

    int _uORB_IMU_Accel_Vector = 0;
    float _uORB_Gryo__Roll = 0;
    float _uORB_Gryo_Pitch = 0;
    float _uORB_Gryo___Yaw = 0;
    float _uORB_Real__Roll = 0;
    float _uORB_Real_Pitch = 0;
    float _uORB_Accel__Roll = 0;
    float _uORB_Accel_Pitch = 0;

    int _flag_MPU9250_G_X_Cali;
    int _flag_MPU9250_G_Y_Cali;
    int _flag_MPU9250_G_Z_Cali;

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

    inline int MPUCalibration(double *AccelCaliData)
    {
        PrivateData._flag_MPU9250_A_X_Scal = AccelCaliData[MPUAccelScalX];
        PrivateData._flag_MPU9250_A_Y_Scal = AccelCaliData[MPUAccelScalY];
        PrivateData._flag_MPU9250_A_Z_Scal = AccelCaliData[MPUAccelScalZ];
        PrivateData._flag_MPU9250_A_X_Cali = AccelCaliData[MPUAccelCaliX];
        PrivateData._flag_MPU9250_A_Y_Cali = AccelCaliData[MPUAccelCaliY];
        PrivateData._flag_MPU9250_A_Z_Cali = AccelCaliData[MPUAccelCaliZ];

        PrivateData._flag_MPU9250_G_X_Cali = 0;
        PrivateData._flag_MPU9250_G_Y_Cali = 0;
        PrivateData._flag_MPU9250_G_Z_Cali = 0;
        PrivateData._uORB_MPU9250_Accel_Static_Vector = 0;
        for (int cali_count = 0; cali_count < 2000; cali_count++)
        {
            IMUSensorsDataRead();
            PrivateData._uORB_MPU9250_A_X = PrivateData._uORB_MPU9250_A_X * PrivateData._flag_MPU9250_A_X_Scal - PrivateData._flag_MPU9250_A_X_Cali;
            PrivateData._uORB_MPU9250_A_Y = PrivateData._uORB_MPU9250_A_Y * PrivateData._flag_MPU9250_A_Y_Scal - PrivateData._flag_MPU9250_A_Y_Cali;
            PrivateData._uORB_MPU9250_A_Z = PrivateData._uORB_MPU9250_A_Z * PrivateData._flag_MPU9250_A_Z_Scal - PrivateData._flag_MPU9250_A_Z_Cali;
            PrivateData._uORB_MPU9250_A_Vector = sqrt((PrivateData._uORB_MPU9250_A_X * PrivateData._uORB_MPU9250_A_X) +
                                                      (PrivateData._uORB_MPU9250_A_Y * PrivateData._uORB_MPU9250_A_Y) +
                                                      (PrivateData._uORB_MPU9250_A_Z * PrivateData._uORB_MPU9250_A_Z));
            PrivateData._flag_MPU9250_G_X_Cali += PrivateData._uORB_MPU9250_G_X;
            PrivateData._flag_MPU9250_G_Y_Cali += PrivateData._uORB_MPU9250_G_Y;
            PrivateData._flag_MPU9250_G_Z_Cali += PrivateData._uORB_MPU9250_G_Z;
            PrivateData._uORB_MPU9250_Accel_Static_Vector += PrivateData._uORB_MPU9250_A_Vector;
            usleep((int)(1.f / (float)MPUUpdateFreq * 1000000.f));
        }
        PrivateData._flag_MPU9250_G_X_Cali = PrivateData._flag_MPU9250_G_X_Cali / 2000.f;
        PrivateData._flag_MPU9250_G_Y_Cali = PrivateData._flag_MPU9250_G_Y_Cali / 2000.f;
        PrivateData._flag_MPU9250_G_Z_Cali = PrivateData._flag_MPU9250_G_Z_Cali / 2000.f;
        PrivateData._uORB_MPU9250_Accel_Static_Vector = PrivateData._uORB_MPU9250_Accel_Static_Vector / 2000.f;
        return 0;
    };

    inline void MPUAccelCalibration(int AccelCali, double *AccelCaliTmp)
    {
        int AccelCaliTmpTotal = 0;
        for (int cali_count = 0; cali_count < 1000; cali_count++)
        {
            IMUSensorsDataRead();
            switch (AccelCali)
            {
            case (MPUAccelNoseUp):
                AccelCaliTmp[AccelCali] = AccelCaliTmp[AccelCali] < PrivateData._uORB_MPU9250_A_Y ? PrivateData._uORB_MPU9250_A_Y : AccelCaliTmp[AccelCali];
                break;
            case MPUAccelNoseDown:
                AccelCaliTmp[AccelCali] = AccelCaliTmp[AccelCali] > PrivateData._uORB_MPU9250_A_Y ? PrivateData._uORB_MPU9250_A_Y : AccelCaliTmp[AccelCali];
                break;
            case MPUAccelNoseRight:
                AccelCaliTmp[AccelCali] = AccelCaliTmp[AccelCali] > PrivateData._uORB_MPU9250_A_X ? PrivateData._uORB_MPU9250_A_X : AccelCaliTmp[AccelCali];
                break;
            case MPUAccelNoseLeft:
                AccelCaliTmp[AccelCali] = AccelCaliTmp[AccelCali] < PrivateData._uORB_MPU9250_A_X ? PrivateData._uORB_MPU9250_A_X : AccelCaliTmp[AccelCali];
                break;
            case MPUAccelNoseTop:
                AccelCaliTmp[AccelCali] = AccelCaliTmp[AccelCali] < PrivateData._uORB_MPU9250_A_Z ? PrivateData._uORB_MPU9250_A_Z : AccelCaliTmp[AccelCali];
                break;
            case MPUAccelNoseRev:
                AccelCaliTmp[AccelCali] = AccelCaliTmp[AccelCali] > PrivateData._uORB_MPU9250_A_Z ? PrivateData._uORB_MPU9250_A_Z : AccelCaliTmp[AccelCali];
                break;
            }
            usleep((int)(1.f / (float)MPUUpdateFreq * 1000000.f));
        }
        if (AccelCali == MPUAccelCaliGet)
        {
            AccelCaliTmp[MPUAccelCaliX] = (AccelCaliTmp[MPUAccelNoseRight] + AccelCaliTmp[MPUAccelNoseLeft]) / 2.f;
            AccelCaliTmp[MPUAccelCaliY] = (AccelCaliTmp[MPUAccelNoseUp] + AccelCaliTmp[MPUAccelNoseDown]) / 2.f;
            AccelCaliTmp[MPUAccelCaliZ] = (AccelCaliTmp[MPUAccelNoseTop] + AccelCaliTmp[MPUAccelNoseRev]) / 2.f;
            AccelCaliTmp[MPUAccelScalX] = MPUGStandard / (AccelCaliTmp[MPUAccelNoseLeft] - AccelCaliTmp[7]);
            AccelCaliTmp[MPUAccelScalY] = MPUGStandard / (AccelCaliTmp[MPUAccelNoseUp] - AccelCaliTmp[8]);
            AccelCaliTmp[MPUAccelScalZ] = MPUGStandard / (AccelCaliTmp[MPUAccelNoseTop] - AccelCaliTmp[9]);
        }
    }

    inline MPUData MPUSensorsDataGet()
    {
        IMUSensorsDataRead();
        PrivateData._uORB_MPU9250_G_X -= PrivateData._flag_MPU9250_G_X_Cali;
        PrivateData._uORB_MPU9250_G_Y -= PrivateData._flag_MPU9250_G_Y_Cali;
        PrivateData._uORB_MPU9250_G_Z -= PrivateData._flag_MPU9250_G_Z_Cali;

        PrivateData._uORB_MPU9250_A_X = PrivateData._uORB_MPU9250_A_X * PrivateData._flag_MPU9250_A_X_Scal - PrivateData._flag_MPU9250_A_X_Cali;
        PrivateData._uORB_MPU9250_A_Y = PrivateData._uORB_MPU9250_A_Y * PrivateData._flag_MPU9250_A_Y_Scal - PrivateData._flag_MPU9250_A_Y_Cali;
        PrivateData._uORB_MPU9250_A_Z = PrivateData._uORB_MPU9250_A_Z * PrivateData._flag_MPU9250_A_Z_Scal - PrivateData._flag_MPU9250_A_Z_Cali;

        PrivateData._uORB_Gryo_Pitch = (PrivateData._uORB_Gryo_Pitch * MPUGLPFAplah) + ((PrivateData._uORB_MPU9250_G_X / MPU9250_LSB) * (1.f - MPUGLPFAplah));
        PrivateData._uORB_Gryo__Roll = (PrivateData._uORB_Gryo__Roll * MPUGLPFAplah) + ((PrivateData._uORB_MPU9250_G_Y / MPU9250_LSB) * (1.f - MPUGLPFAplah));
        PrivateData._uORB_Gryo___Yaw = (PrivateData._uORB_Gryo___Yaw * MPUGLPFAplah) + ((PrivateData._uORB_MPU9250_G_Z / MPU9250_LSB) * (1.f - MPUGLPFAplah));

        PrivateData._uORB_Real_Pitch += (PrivateData._uORB_MPU9250_G_X / MPU9250_LSB) / MPUUpdateFreq;
        PrivateData._uORB_Real__Roll += (PrivateData._uORB_MPU9250_G_Y / MPU9250_LSB) / MPUUpdateFreq;
        PrivateData._uORB_Real_Pitch += PrivateData._uORB_Real__Roll * sin((PrivateData._uORB_MPU9250_G_Z / MPUUpdateFreq / MPU9250_LSB) * (PI / 180.f));
        PrivateData._uORB_Real__Roll -= PrivateData._uORB_Real_Pitch * sin((PrivateData._uORB_MPU9250_G_Z / MPUUpdateFreq / MPU9250_LSB) * (PI / 180.f));

        PrivateData._uORB_MPU9250_AF_X += (PrivateData._uORB_MPU9250_A_X - PrivateData._uORB_MPU9250_AF_X) * MPUALPFAplah;
        PrivateData._uORB_MPU9250_AF_Y += (PrivateData._uORB_MPU9250_A_Y - PrivateData._uORB_MPU9250_AF_Y) * MPUALPFAplah;
        PrivateData._uORB_MPU9250_AF_Z += (PrivateData._uORB_MPU9250_A_Z - PrivateData._uORB_MPU9250_AF_Z) * MPUALPFAplah;
        MPU9250_Accel_Square_X_Total -= MPU9250_Accel_Square_X[MPU9250_Accel_Clock];
        MPU9250_Accel_Square_X[MPU9250_Accel_Clock] = PrivateData._uORB_MPU9250_AF_X;
        MPU9250_Accel_Square_X_Total += MPU9250_Accel_Square_X[MPU9250_Accel_Clock];
        PrivateData._uORB_MPU9250_AFQ_X = MPU9250_Accel_Square_X_Total / 30.f;
        MPU9250_Accel_Square_Y_Total -= MPU9250_Accel_Square_Y[MPU9250_Accel_Clock];
        MPU9250_Accel_Square_Y[MPU9250_Accel_Clock] = PrivateData._uORB_MPU9250_AF_Y;
        MPU9250_Accel_Square_Y_Total += MPU9250_Accel_Square_Y[MPU9250_Accel_Clock];
        PrivateData._uORB_MPU9250_AFQ_Y = MPU9250_Accel_Square_Y_Total / 30.f;
        MPU9250_Accel_Square_Z_Total -= MPU9250_Accel_Square_Z[MPU9250_Accel_Clock];
        MPU9250_Accel_Square_Z[MPU9250_Accel_Clock] = PrivateData._uORB_MPU9250_AF_Z;
        MPU9250_Accel_Square_Z_Total += MPU9250_Accel_Square_Z[MPU9250_Accel_Clock];
        PrivateData._uORB_MPU9250_AFQ_Z = MPU9250_Accel_Square_Z_Total / 30.f;

        MPU9250_Accel_Clock++;
        if (MPU9250_Accel_Clock == 30)
        {
            MPU9250_Accel_Square2_X_Total -= MPU9250_Accel_Square2_X[MPU9250_Accel_Clock2];
            MPU9250_Accel_Square2_X[MPU9250_Accel_Clock2] = PrivateData._uORB_MPU9250_AFQ_X;
            MPU9250_Accel_Square2_X_Total += MPU9250_Accel_Square2_X[MPU9250_Accel_Clock2];
            PrivateData._uORB_MPU9250_AFQ2_X = MPU9250_Accel_Square2_X_Total / 20.f;
            MPU9250_Accel_Square2_Y_Total -= MPU9250_Accel_Square2_Y[MPU9250_Accel_Clock2];
            MPU9250_Accel_Square2_Y[MPU9250_Accel_Clock2] = PrivateData._uORB_MPU9250_AFQ_Y;
            MPU9250_Accel_Square2_Y_Total += MPU9250_Accel_Square2_Y[MPU9250_Accel_Clock2];
            PrivateData._uORB_MPU9250_AFQ2_Y = MPU9250_Accel_Square2_Y_Total / 20.f;
            MPU9250_Accel_Square2_Z_Total -= MPU9250_Accel_Square2_Z[MPU9250_Accel_Clock2];
            MPU9250_Accel_Square2_Z[MPU9250_Accel_Clock2] = PrivateData._uORB_MPU9250_AFQ_Z;
            MPU9250_Accel_Square2_Z_Total += MPU9250_Accel_Square2_Z[MPU9250_Accel_Clock2];
            PrivateData._uORB_MPU9250_AFQ2_Z = MPU9250_Accel_Square2_Z_Total / 20.f;
            MPU9250_Accel_Clock2++;
            if (MPU9250_Accel_Clock2 == 20)
                MPU9250_Accel_Clock2 = 0;

            MPU9250_Accel_Clock = 0;
        }

        PrivateData._uORB_MPU9250_A_Vector = sqrt((PrivateData._uORB_MPU9250_AFQ_X * PrivateData._uORB_MPU9250_AFQ_X) +
                                                  (PrivateData._uORB_MPU9250_AFQ_Y * PrivateData._uORB_MPU9250_AFQ_Y) +
                                                  (PrivateData._uORB_MPU9250_AFQ_Z * PrivateData._uORB_MPU9250_AFQ_Z));
        PrivateData._uORB_Accel_Pitch = atan2((float)PrivateData._uORB_MPU9250_AFQ_Y, PrivateData._uORB_MPU9250_AFQ_Z) * 180 / PI;
        PrivateData._uORB_Accel__Roll = atan2((float)PrivateData._uORB_MPU9250_AFQ_X, PrivateData._uORB_MPU9250_AFQ_Z) * 180 / PI;
        PrivateData._uORB_MPU9250_Accel_To_Static_X = (sin(PrivateData._uORB_Real__Roll * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector) - PrivateData._uORB_MPU9250_AFQ_X;
        PrivateData._uORB_MPU9250_Accel_To_Static_Y = (sin(PrivateData._uORB_Real_Pitch * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector) - PrivateData._uORB_MPU9250_AFQ_Y;
        PrivateData._uORB_MPU9250_Accel_To_Static_Z = sqrt(PrivateData._uORB_MPU9250_Accel_Static_Vector * PrivateData._uORB_MPU9250_Accel_Static_Vector -
                                                           (sin(PrivateData._uORB_Real__Roll * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector) *
                                                               (sin(PrivateData._uORB_Real__Roll * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector) -
                                                           (sin(PrivateData._uORB_Real_Pitch * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector) *
                                                               (sin(PrivateData._uORB_Real_Pitch * (PI / 180.f)) * PrivateData._uORB_MPU9250_Accel_Static_Vector)) -
                                                      PrivateData._uORB_MPU9250_AFQ_Z;
        PrivateData._uORB_MPU9250_Accel_Static_Angle_X = atan2(abs(PrivateData._uORB_MPU9250_Accel_To_Static_X), abs(PrivateData._uORB_MPU9250_Accel_To_Static_Z)) * 180 / PI;
        PrivateData._uORB_MPU9250_Accel_Static_Angle_Y = atan2(abs(PrivateData._uORB_MPU9250_Accel_To_Static_Y), abs(PrivateData._uORB_MPU9250_Accel_To_Static_Z)) * 180 / PI;

        PrivateData._uORB_MPU9250_Accel_Static_X_Vector = PrivateData._uORB_MPU9250_Accel_To_Static_X / sin(PrivateData._uORB_MPU9250_Accel_Static_Angle_X * (PI / 180.f));
        PrivateData._uORB_MPU9250_Accel_Static_Y_Vector = PrivateData._uORB_MPU9250_Accel_To_Static_Y / sin(PrivateData._uORB_MPU9250_Accel_Static_Angle_Y * (PI / 180.f));

        PrivateData._uORB_MPU9250_A_Static_X = PrivateData._uORB_MPU9250_Accel_Static_X_Vector *
                                               sin((abs(PrivateData._uORB_MPU9250_Accel_Static_Angle_X) + abs(PrivateData._uORB_Real__Roll)) * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_Y = PrivateData._uORB_MPU9250_Accel_Static_Y_Vector *
                                               sin((abs(PrivateData._uORB_MPU9250_Accel_Static_Angle_Y) + abs(PrivateData._uORB_Real_Pitch)) * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_Z = PrivateData._uORB_MPU9250_Accel_Static_X_Vector * cos((PrivateData._uORB_MPU9250_Accel_Static_Angle_X + abs(PrivateData._uORB_Real__Roll)) * (PI / 180.f));
        PrivateData._uORB_MPU9250_A_Static_Z += PrivateData._uORB_MPU9250_Accel_Static_Y_Vector * cos((PrivateData._uORB_MPU9250_Accel_Static_Angle_Y + abs(PrivateData._uORB_Real_Pitch)) * (PI / 180.f));

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
    float MPU9250_LSB = 65.5;
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

    int MPU9250_Accel_Square_X[60] = {0};
    double MPU9250_Accel_Square_X_Total = 0;
    int MPU9250_Accel_Square_Y[60] = {0};
    double MPU9250_Accel_Square_Y_Total = 0;
    int MPU9250_Accel_Square_Z[60] = {0};
    double MPU9250_Accel_Square_Z_Total = 0;
    int MPU9250_Accel_Clock = 0;

    int MPU9250_Accel_Square2_X[100] = {0};
    double MPU9250_Accel_Square2_X_Total = 0;
    int MPU9250_Accel_Square2_Y[100] = {0};
    double MPU9250_Accel_Square2_Y_Total = 0;
    int MPU9250_Accel_Square2_Z[100] = {0};
    double MPU9250_Accel_Square2_Z_Total = 0;
    int MPU9250_Accel_Clock2 = 0;
};