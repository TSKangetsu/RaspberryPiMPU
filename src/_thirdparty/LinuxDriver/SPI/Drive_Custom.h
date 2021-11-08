#pragma once

inline int _s_spiOpen(int spiChan, int spiBaud, int spiFlags)
{
    return -1;
}

inline int _s_spiClose(int fd)
{
    return -1;
}

inline int _s_spiRead(int fd, void *buffer, int speed, int count)
{
    return -1;
}

inline int _s_spiWrite(int fd, void *buffer, int speed, int count)
{
    return -1;
}

inline int _s_spiXfer(int fd, void *txBuf, void *rxBuf, int speed, int count)
{
    return -1;
}