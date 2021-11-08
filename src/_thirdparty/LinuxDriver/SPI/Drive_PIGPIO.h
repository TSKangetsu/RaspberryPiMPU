#pragma once
#include <pigpio.h>
#include<stdlib.h>

inline int _s_spiOpen(const char *spiChan, int spiBaud, int spiFlags)
{
    gpioInitialise();
    return spiOpen(atoi(spiChan), spiBaud, spiFlags);
}

inline int _s_spiClose(int fd)
{
    return spiClose(fd);
}

inline int _s_spiRead(int fd, void *buffer, int speed, int count)
{
    return spiRead(fd, (char *)buffer, count);
}

inline int _s_spiWrite(int fd, void *buffer, int speed, int count)
{
    return spiWrite(fd, (char *)buffer, count);
}

inline int _s_spiXfer(int fd, void *txBuf, void *rxBuf, int speed, int count)
{
    return spiXfer(fd, (char *)txBuf, (char *)rxBuf, count);
}