#include <math.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#define MPUTypeI2C 0
#define MPUTypeSPI 1

#define MPULPFAplah 0.02

#define MPUMixKalman 1
#define MPUMixTradition 0

#define MPUMixTraditionAplah 0.996

struct MPUData
{
    int _uORB_MPU9250_A_X = 0;
    int _uORB_MPU9250_A_Y = 0;
    int _uORB_MPU9250_A_Z = 0;
    int _uORB_MPU9250_AF_X = 0;
    int _uORB_MPU9250_AF_Y = 0;
    int _uORB_MPU9250_AF_Z = 0;
    int _uORB_MPU9250_AFQ_X = 0;
    int _uORB_MPU9250_AFQ_Y = 0;
    int _uORB_MPU9250_AFQ_Z = 0;
    int _uORB_MPU9250_G_X = 0;
    int _uORB_MPU9250_G_Y = 0;
    int _uORB_MPU9250_G_Z = 0;
    int _uORB_MPU9250_M_X = 0;
    int _uORB_MPU9250_M_Y = 0;
    int _uORB_MPU9250_M_Z = 0;

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

    inline int MPUGryoCalibration()
    {
        PrivateData._flag_MPU9250_G_X_Cali = 0;
        PrivateData._flag_MPU9250_G_Y_Cali = 0;
        PrivateData._flag_MPU9250_G_Z_Cali = 0;
        for (int cali_count = 0; cali_count < 2000; cali_count++)
        {
            IMUSensorsDataRead();
            PrivateData._flag_MPU9250_G_X_Cali += PrivateData._uORB_MPU9250_G_X;
            PrivateData._flag_MPU9250_G_Y_Cali += PrivateData._uORB_MPU9250_G_Y;
            PrivateData._flag_MPU9250_G_Z_Cali += PrivateData._uORB_MPU9250_G_Z;
            usleep(500);
        }
        PrivateData._flag_MPU9250_G_X_Cali = PrivateData._flag_MPU9250_G_X_Cali / 2000;
        PrivateData._flag_MPU9250_G_Y_Cali = PrivateData._flag_MPU9250_G_Y_Cali / 2000;
        PrivateData._flag_MPU9250_G_Z_Cali = PrivateData._flag_MPU9250_G_Z_Cali / 2000;

        return 0;
    };

    inline MPUData MPUSensorsDataGet()
    {
        IMUSensorsDataRead();
        PrivateData._uORB_MPU9250_G_X -= PrivateData._flag_MPU9250_G_X_Cali;
        PrivateData._uORB_MPU9250_G_Y -= PrivateData._flag_MPU9250_G_Y_Cali;
        PrivateData._uORB_MPU9250_G_Z -= PrivateData._flag_MPU9250_G_Z_Cali;

        PrivateData._uORB_Gryo_Pitch = (PrivateData._uORB_Gryo_Pitch * 0.7) + ((PrivateData._uORB_MPU9250_G_X / MPU9250_LSB) * 0.3);
        PrivateData._uORB_Gryo__Roll = (PrivateData._uORB_Gryo__Roll * 0.7) + ((PrivateData._uORB_MPU9250_G_Y / MPU9250_LSB) * 0.3);
        PrivateData._uORB_Gryo___Yaw = (PrivateData._uORB_Gryo___Yaw * 0.7) + ((PrivateData._uORB_MPU9250_G_Z / MPU9250_LSB) * 0.3);

        PrivateData._uORB_Real_Pitch += (PrivateData._uORB_MPU9250_G_X / MPU9250_LSB) / MPUUpdateFreq;
        PrivateData._uORB_Real__Roll += (PrivateData._uORB_MPU9250_G_Y / MPU9250_LSB) / MPUUpdateFreq;
        PrivateData._uORB_Real_Pitch -= PrivateData._uORB_Real__Roll * sin((PrivateData._uORB_Gryo___Yaw / MPUUpdateFreq / MPU9250_LSB) * (3.14 / 180));
        PrivateData._uORB_Real__Roll += PrivateData._uORB_Real_Pitch * sin((PrivateData._uORB_Gryo___Yaw / MPUUpdateFreq / MPU9250_LSB) * (3.14 / 180));

        PrivateData._uORB_MPU9250_AF_X += (PrivateData._uORB_MPU9250_A_X - PrivateData._uORB_MPU9250_AF_X) * MPULPFAplah;
        PrivateData._uORB_MPU9250_AF_Y += (PrivateData._uORB_MPU9250_A_Y - PrivateData._uORB_MPU9250_AF_Y) * MPULPFAplah;
        PrivateData._uORB_MPU9250_AF_Z += (PrivateData._uORB_MPU9250_A_Z - PrivateData._uORB_MPU9250_AF_Z) * MPULPFAplah;
        MPU9250_Accel_Square_X_Total -= MPU9250_Accel_Square_X[MPU9250_Accel_Clock];
        MPU9250_Accel_Square_X[MPU9250_Accel_Clock] = PrivateData._uORB_MPU9250_AF_X;
        MPU9250_Accel_Square_X_Total += MPU9250_Accel_Square_X[MPU9250_Accel_Clock];
        PrivateData._uORB_MPU9250_AFQ_X = MPU9250_Accel_Square_X_Total / 50.f;
        MPU9250_Accel_Square_Y_Total -= MPU9250_Accel_Square_Y[MPU9250_Accel_Clock];
        MPU9250_Accel_Square_Y[MPU9250_Accel_Clock] = PrivateData._uORB_MPU9250_AF_Y;
        MPU9250_Accel_Square_Y_Total += MPU9250_Accel_Square_Y[MPU9250_Accel_Clock];
        PrivateData._uORB_MPU9250_AFQ_Y = MPU9250_Accel_Square_Y_Total / 50.f;
        MPU9250_Accel_Square_Z_Total -= MPU9250_Accel_Square_Z[MPU9250_Accel_Clock];
        MPU9250_Accel_Square_Z[MPU9250_Accel_Clock] = PrivateData._uORB_MPU9250_AF_Z;
        MPU9250_Accel_Square_Z_Total += MPU9250_Accel_Square_Z[MPU9250_Accel_Clock];
        PrivateData._uORB_MPU9250_AFQ_Z = MPU9250_Accel_Square_Z_Total / 50.f;
        MPU9250_Accel_Clock++;
        if (MPU9250_Accel_Clock == 50)
            MPU9250_Accel_Clock = 0;

        PrivateData._uORB_IMU_Accel_Vector = sqrt((PrivateData._uORB_MPU9250_AFQ_X * PrivateData._uORB_MPU9250_AFQ_X) + (PrivateData._uORB_MPU9250_AFQ_Y * PrivateData._uORB_MPU9250_AFQ_Y) + (PrivateData._uORB_MPU9250_AFQ_Z * PrivateData._uORB_MPU9250_AFQ_Z));
        if (abs(PrivateData._uORB_MPU9250_AFQ_X) < PrivateData._uORB_IMU_Accel_Vector)
            PrivateData._uORB_Accel__Roll = asin((float)PrivateData._uORB_MPU9250_AFQ_X / PrivateData._uORB_IMU_Accel_Vector) * -57.296;
        if (abs(PrivateData._uORB_MPU9250_AFQ_Y) < PrivateData._uORB_IMU_Accel_Vector)
            PrivateData._uORB_Accel_Pitch = asin((float)PrivateData._uORB_MPU9250_AFQ_Y / PrivateData._uORB_IMU_Accel_Vector) * 57.296;

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
            PrivateData._uORB_MPU9250_A_X = (short)(Tmp_MPU9250_Buffer[0] << 8 | Tmp_MPU9250_Buffer[1]);
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
            PrivateData._uORB_MPU9250_A_X = (short)((int)Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)Tmp_MPU9250_SPI_Buffer[2]);
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
};