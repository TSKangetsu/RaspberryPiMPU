#pragma once
#include <iostream>
#include "../MPU.hpp"
#include "../_thirdparty/LinuxDriver/SPI/Drive_LinuxSPI.h"

inline void ICM20602Init(MPUConfig &PrivateConfig, MPUData &PrivateData, int &Sensor_fd)
{
    if (PrivateConfig.MPUType == MPUTypeSPI)
    {
        Sensor_fd = _s_spiOpen(PrivateConfig.ICMSPIChannel, PrivateConfig.MPU9250_SPI_Freq, 0);
        if (Sensor_fd < 0)
            throw std::invalid_argument("[SPI] ICM device can't open");
        uint8_t ICM20602_SPI_Config_WHOAMI[2] = {0xf5, 0x00};
        _s_spiXfer(Sensor_fd, ICM20602_SPI_Config_WHOAMI, ICM20602_SPI_Config_WHOAMI, PrivateConfig.MPU9250_SPI_Freq, 2);
        PrivateData.DeviceType = ICM20602_SPI_Config_WHOAMI[1];

        uint8_t ICM20602_SPI_Config_RESET[2] = {0x6b, 0x80};
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_RESET, PrivateConfig.MPU9250_SPI_Freq, 2); // reset
        usleep(500);
        uint8_t ICM20602_SPI_Config_RESET2[2] = {0x68, 0x03};
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_RESET2, PrivateConfig.MPU9250_SPI_Freq, 2); // BIT_ACC | BIT_TEMP reset
        usleep(500);
        uint8_t ICM20602_SPI_Config_RESET3[2] = {0x6b, 0x00};
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_RESET3, PrivateConfig.MPU9250_SPI_Freq, 2); // reset
        usleep(500);
        uint8_t ICM20602_SPI_Config_RESET4[2] = {0x6b, 0x01};
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_RESET4, PrivateConfig.MPU9250_SPI_Freq, 2); // reset
        usleep(1000);
        uint8_t ICM20602_SPI_Config_RESET5[2] = {0X6C, 0x00};
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_RESET4, PrivateConfig.MPU9250_SPI_Freq, 2); //  acc | gyro on
        usleep(1000);

        uint8_t ICM20602_SPI_Config_ALPF[2] = {0x1d, 0x00};                                  // FChoice 1, DLPF 3 , dlpf cut off 44.8hz for accel is 0x03, but now 0x00 is not apply accel hardware
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_ALPF, PrivateConfig.MPU9250_SPI_Freq, 2); // Accel2
        usleep(15);
        uint8_t ICM20602_SPI_Config_Acce[2] = {0x1c, 0x18};                                  // Full AccelScale +- 16g
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_Acce, PrivateConfig.MPU9250_SPI_Freq, 2); // Accel
        usleep(15);
        uint8_t ICM20602_SPI_Config_Gyro[2] = {0x1b, 0x18};                                  // Full GyroScale +-2000dps, dlpf 250hz
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_Gyro, PrivateConfig.MPU9250_SPI_Freq, 2); // Gryo
        usleep(15);

        uint8_t ICM20602_SPI_Config_GLPF[2] = {0x1a, 0x00};                                  // DLPF_CFG is 000 , with Gyro dlpf is 250hz
        _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_GLPF, PrivateConfig.MPU9250_SPI_Freq, 2); // config
        usleep(15);

        // uint8_t ICM20602_SPI_Config_INTC[2] = {0x37, 0x22};
        // _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_INTC, PrivateConfig.ICM20602_SPI_Freq, 2);
        // usleep(500);
        // uint8_t ICM20602_SPI_Config_INTE[2] = {0x38, 0x01};
        // _s_spiWrite(Sensor_fd, ICM20602_SPI_Config_INTE, PrivateConfig.ICM20602_SPI_Freq, 2);
        // usleep(500);
    }
    else if (PrivateConfig.MPUType == MPUTypeI2C)
    {
    }
}



inline void ICM20602DataSPIRead(int &Sensor_fd,int *Six_AxisData,int ICM20602_SPI_Freq)
{

    uint8_t Tmp_ICM20602_SPI_BufferX[2] = {0};
    Tmp_ICM20602_SPI_BufferX[0] = 0xBA;
    _s_spiXfer(Sensor_fd, Tmp_ICM20602_SPI_BufferX, Tmp_ICM20602_SPI_BufferX, ICM20602_SPI_Freq, 2);

    if (Tmp_ICM20602_SPI_BufferX[1] & 0x01)
    {

        uint8_t Tmp_ICM20602_SPI_Buffer[8] = {0};
        uint8_t Tmp_ICM20602_SPI_Bufferout[8] = {0};
        Tmp_ICM20602_SPI_Buffer[0] = 0xBB;
        {
            _s_spiXfer(Sensor_fd, Tmp_ICM20602_SPI_Buffer, Tmp_ICM20602_SPI_Bufferout, ICM20602_SPI_Freq, 8);
            Six_AxisData[0] = (short)((int)Tmp_ICM20602_SPI_Bufferout[1] << 8 | (int)Tmp_ICM20602_SPI_Bufferout[2]);
            Six_AxisData[1] = (short)((int)Tmp_ICM20602_SPI_Bufferout[3] << 8 | (int)Tmp_ICM20602_SPI_Bufferout[4]);
            Six_AxisData[2] = (short)((int)Tmp_ICM20602_SPI_Bufferout[5] << 8 | (int)Tmp_ICM20602_SPI_Bufferout[6]);
        }

        {
            uint8_t Tmp_ICM20602_SPI_GBuffer[8] = {0};
            uint8_t Tmp_ICM20602_SPI_GBufferout[8] = {0};
            Tmp_ICM20602_SPI_GBuffer[0] = 0xC3;
            _s_spiXfer(Sensor_fd, Tmp_ICM20602_SPI_GBuffer, Tmp_ICM20602_SPI_GBufferout, ICM20602_SPI_Freq, 8);
            Six_AxisData[3] = (short)((int)Tmp_ICM20602_SPI_GBufferout[1] << 8 | (int)Tmp_ICM20602_SPI_GBufferout[2]);
            Six_AxisData[4] = (short)((int)Tmp_ICM20602_SPI_GBufferout[3] << 8 | (int)Tmp_ICM20602_SPI_GBufferout[4]);
            Six_AxisData[5] = (short)((int)Tmp_ICM20602_SPI_GBufferout[5] << 8 | (int)Tmp_ICM20602_SPI_GBufferout[6]);
        }
    }
}