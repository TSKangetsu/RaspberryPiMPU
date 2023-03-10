#pragma once
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <asm/ioctls.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

inline int _s_spiOpen(const char *Device, int spiBaud, int spiFlags)
{
    int i, fd;
    char spiMode;
    char spiBits = 8;
    char dev[32];

    spiMode = spiFlags & 3;
    spiBits = 8;

    if ((fd = open(Device, O_RDWR)) < 0)
    {
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_MODE, &spiMode) < 0)
    {
        close(fd);
        return -2;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spiBits) < 0)
    {
        close(fd);
        return -3;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spiBaud) < 0)
    {
        close(fd);
        return -4;
    }

    return fd;
}

inline int _s_spiRead(int fd, void *buffer, int speed, int count)
{
    int err;
    struct spi_ioc_transfer spi;

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long)NULL;
    spi.rx_buf = (unsigned long)buffer;
    spi.len = count;
    spi.speed_hz = speed;
    spi.delay_usecs = 0;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    err = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

    return err;
}

inline int _s_spiWrite(int fd, void *buffer, int speed, int count)
{
    int err;
    struct spi_ioc_transfer spi;

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long)buffer;
    spi.rx_buf = (unsigned long)NULL;
    spi.len = count;
    spi.speed_hz = speed;
    spi.delay_usecs = 0;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    err = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

    return err;
}

inline int _s_spiXfer(int fd, uint8_t *txBuf, uint8_t *rxBuf, int speed, int count)
{
    int err;
    struct spi_ioc_transfer spi;

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long)txBuf;
    spi.rx_buf = (unsigned long)rxBuf;
    spi.len = count;
    spi.speed_hz = speed;
    spi.delay_usecs = 0;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    err = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

    return err;
}

inline int _s_spiClose(int fd)
{
    return close(fd);
}