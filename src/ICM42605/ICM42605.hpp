#pragma once
#include <iostream>
#include "../MPU.hpp"
#include "../_thirdparty/LinuxDriver/SPI/Drive_LinuxSPI.h"

inline void ICM42605Init(MPUConfig &PrivateConfig, MPUData &PrivateData, int &Sensor_fd)
{
    if (PrivateConfig.MPUType == MPUTypeSPI)
    {
        uint8_t ICM42605_SPI_Config_WHOAMI[2] = {0xf5, 0x00};
        _s_spiXfer(Sensor_fd, ICM42605_SPI_Config_WHOAMI, ICM42605_SPI_Config_WHOAMI, PrivateConfig.MPU9250_SPI_Freq, 2);
        PrivateData.DeviceType = ICM42605_SPI_Config_WHOAMI[1];

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

inline void ICM42605DataSPIRead(int &Sensor_fd, int *Six_AxisData, int ICM42605_SPI_Freq)
{
    uint8_t Tmp_ICM42605_SPI_BufferX[2] = {0};
    Tmp_ICM42605_SPI_BufferX[0] = 0xAD;
    _s_spiXfer(Sensor_fd, Tmp_ICM42605_SPI_BufferX, Tmp_ICM42605_SPI_BufferX, ICM42605_SPI_Freq, 2);

    if (Tmp_ICM42605_SPI_BufferX[1] & 0x08)
    {
        uint8_t Tmp_ICM42605_SPI_Buffer[8] = {0};
        uint8_t Tmp_ICM42605_SPI_Bufferout[8] = {0};
        Tmp_ICM42605_SPI_Buffer[0] = 0x9f;
        {
            _s_spiXfer(Sensor_fd, Tmp_ICM42605_SPI_Buffer, Tmp_ICM42605_SPI_Bufferout, ICM42605_SPI_Freq, 8);
            Six_AxisData[0] = (short)((int)Tmp_ICM42605_SPI_Bufferout[1] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[2]);
            Six_AxisData[1] = (short)((int)Tmp_ICM42605_SPI_Bufferout[3] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[4]);
            Six_AxisData[2] = (short)((int)Tmp_ICM42605_SPI_Bufferout[5] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[6]);
        }

        {
            uint8_t Tmp_ICM42605_SPI_GBuffer[8] = {0};
            uint8_t Tmp_ICM42605_SPI_GBufferout[8] = {0};
            Tmp_ICM42605_SPI_GBuffer[0] = 0xa5;
            _s_spiXfer(Sensor_fd, Tmp_ICM42605_SPI_GBuffer, Tmp_ICM42605_SPI_GBufferout, ICM42605_SPI_Freq, 8);
            Six_AxisData[3] = (short)((int)Tmp_ICM42605_SPI_GBufferout[1] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[2]);
            Six_AxisData[4] = (short)((int)Tmp_ICM42605_SPI_GBufferout[3] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[4]);
            Six_AxisData[5] = (short)((int)Tmp_ICM42605_SPI_GBufferout[5] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[6]);
        }
    }
}
