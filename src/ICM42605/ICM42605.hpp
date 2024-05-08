#pragma once
#include <iostream>
#include "../MPU.hpp"
#include "../_thirdparty/LinuxDriver/SPI/Drive_LinuxSPI.h"

inline void ICM42605Init(MPUConfig &PrivateConfig, MPUData &PrivateData, int &Sensor_fd)
{
    if (PrivateConfig.MPUType == MPUTypeSPI)
    {
        Sensor_fd = _s_spiOpen(PrivateConfig.ICMSPIChannel, PrivateConfig.MPU9250_SPI_Freq, 0);
        if (Sensor_fd < 0)
            throw std::invalid_argument("[SPI] ICM device can't open");
        uint8_t ICM42605_SPI_Config_WHOAMI[2] = {0xf5, 0x00};
        _s_spiXfer(Sensor_fd, ICM42605_SPI_Config_WHOAMI, ICM42605_SPI_Config_WHOAMI, PrivateConfig.MPU9250_SPI_Freq, 2);
        PrivateData.DeviceType = ICM42605_SPI_Config_WHOAMI[1];
        std::cout << "Device Type: " << std::dec << static_cast<int>(PrivateData.DeviceType) << std::endl;

        //---------------------reset--------------------------//
        uint8_t ICM42605_SPI_Config_RESET[2] = {0x76, 0x00};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_RESET, PrivateConfig.MPU9250_SPI_Freq, 2); // bank0
        usleep(500);
        uint8_t ICM42605_SPI_Config_RESET2[2] = {0x11, 0x01};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_RESET2, PrivateConfig.MPU9250_SPI_Freq, 2); // soft reset
        usleep(500);

        uint8_t ICM42605_SPI_Config_CONF[2] = {0x76, 0x01};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF, PrivateConfig.MPU9250_SPI_Freq, 2); // bank1
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF2[2] = {0x11, 0x02};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF2, PrivateConfig.MPU9250_SPI_Freq, 2); // 4 wire spi mode
        usleep(500);

        uint8_t ICM42605_SPI_Config_CONF3[2] = {0x76, 0x00};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.MPU9250_SPI_Freq, 2); // bank0
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF4[2] = {0x16, 0x40};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF4, PrivateConfig.MPU9250_SPI_Freq, 2); // Stream-to-FIFO Mode
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF5[2] = {0xe5, 0x00};
        _s_spiXfer(Sensor_fd, ICM42605_SPI_Config_CONF5, ICM42605_SPI_Config_CONF5, PrivateConfig.MPU9250_SPI_Freq, 2);
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF6[2] = {0x60, 0x00};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF6, PrivateConfig.MPU9250_SPI_Freq, 2); // watermark
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF7[2] = {0x61, 0x02};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF7, PrivateConfig.MPU9250_SPI_Freq, 2); // watermark
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF8[2] = {0x65, ICM42605_SPI_Config_CONF5[1]};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF8, PrivateConfig.MPU9250_SPI_Freq, 2);
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF9[2] = {0x5f, 0x63};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF9, PrivateConfig.MPU9250_SPI_Freq, 2);
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF10[2] = {0x5f, 0x63};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF10, PrivateConfig.MPU9250_SPI_Freq, 2); // Enable the accel and gyro to the FIFO
        usleep(500);

        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.MPU9250_SPI_Freq, 2); // bank0
        usleep(500);
        uint8_t ICM42605_SPI_Config_CONF11[2] = {0x14, 0x36};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF11, PrivateConfig.MPU9250_SPI_Freq, 2);
        usleep(500);

        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.MPU9250_SPI_Freq, 2); // bank0
        usleep(500);
        uint8_t ICM42605_SPI_Config_CON12[2] = {0xe5, 0x00};
        _s_spiXfer(Sensor_fd, ICM42605_SPI_Config_CON12, ICM42605_SPI_Config_CON12, PrivateConfig.MPU9250_SPI_Freq, 2);
        usleep(500);
        ICM42605_SPI_Config_CON12[1] |= (1 << 2); // FIFO_THS_INT1_ENABLE
        uint8_t ICM42605_SPI_Config_CONF13[2] = {0x65, ICM42605_SPI_Config_CON12[1]};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF11, PrivateConfig.MPU9250_SPI_Freq, 2);
        usleep(500);

        uint8_t ICM42605_SPI_Config_CON13[2] = {0x2d, 0x08};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CON13, PrivateConfig.MPU9250_SPI_Freq, 2); // DATA_RDY_INT
        usleep(500);

        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.MPU9250_SPI_Freq, 2); // bank0
        usleep(500);
        uint8_t ICM42605_SPI_Config_Accel[2] = {0x50, 0x07};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_Accel, PrivateConfig.MPU9250_SPI_Freq, 2); // 16g || 500hz lp
        usleep(500);

        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.MPU9250_SPI_Freq, 2); // bank0
        usleep(500);
        uint8_t ICM42605_SPI_Config_Gyro[2] = {0x4f, 0x07};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_Gyro, PrivateConfig.MPU9250_SPI_Freq, 2); // 2000dps || 2khz
        usleep(500);

        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.MPU9250_SPI_Freq, 2); // bank0
        usleep(500);
        uint8_t ICM42605_SPI_Config_Tem[2] = {0x4e, 0x0f};
        _s_spiWrite(Sensor_fd, ICM42605_SPI_Config_Tem, PrivateConfig.MPU9250_SPI_Freq, 2); // Accel on in LNM ||  Gyro on in LNM
        usleep(200000);
    }
    else if (PrivateConfig.MPUType == MPUTypeI2C)
    {
    }
}

inline void ICM42605DataRead(MPUConfig &PrivateConfig, MPUData &PrivateData, int &Sensor_fd)
{
    if (PrivateConfig.MPUType == MPUTypeI2C)
    {
        //
    }
    else if (PrivateConfig.MPUType == MPUTypeSPI)
    {
        uint8_t Tmp_ICM42605_SPI_BufferX[2] = {0};
        Tmp_ICM42605_SPI_BufferX[0] = 0xAD;
        _s_spiXfer(Sensor_fd, Tmp_ICM42605_SPI_BufferX, Tmp_ICM42605_SPI_BufferX, PrivateConfig.MPU9250_SPI_Freq, 2);

        if (Tmp_ICM42605_SPI_BufferX[1] & 0x08)
        {

            uint8_t Tmp_ICM42605_SPI_Buffer[8] = {0};
            uint8_t Tmp_ICM42605_SPI_Bufferout[8] = {0};
            Tmp_ICM42605_SPI_Buffer[0] = 0x9f;
            if (PrivateData._uORB_MPU9250_AccelCountDown >= (PrivateConfig.TargetFreqency / PrivateConfig.AccTargetFreqency))
            {
                _s_spiXfer(Sensor_fd, Tmp_ICM42605_SPI_Buffer, Tmp_ICM42605_SPI_Bufferout, PrivateConfig.MPU9250_SPI_Freq, 8);
                int Tmp_AX = (short)((int)Tmp_ICM42605_SPI_Bufferout[1] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[2]);
                int Tmp_AY = (short)((int)Tmp_ICM42605_SPI_Bufferout[3] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[4]);
                int Tmp_AZ = (short)((int)Tmp_ICM42605_SPI_Bufferout[5] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[6]);
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
                uint8_t Tmp_ICM42605_SPI_GBuffer[8] = {0};
                uint8_t Tmp_ICM42605_SPI_GBufferout[8] = {0};
                Tmp_ICM42605_SPI_GBuffer[0] = 0xa5;
                _s_spiXfer(Sensor_fd, Tmp_ICM42605_SPI_GBuffer, Tmp_ICM42605_SPI_GBufferout, PrivateConfig.MPU9250_SPI_Freq, 8);
                int Tmp_GX = (short)((int)Tmp_ICM42605_SPI_GBufferout[1] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[2]);
                int Tmp_GY = (short)((int)Tmp_ICM42605_SPI_GBufferout[3] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[4]);
                int Tmp_GZ = (short)((int)Tmp_ICM42605_SPI_GBufferout[5] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[6]);
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