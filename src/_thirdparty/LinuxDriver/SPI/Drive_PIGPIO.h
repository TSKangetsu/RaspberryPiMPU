#pragma once

inline int spiOpen(int spiChan,int  spiBaud, int  spiFlags)
{
    return -1;
}

inline int spiClose(int fd)
{
    return -1;
}

inline int spiRead(int fd, void *buffer, int count)
{
    return -1;
}

inline int spiWrite(int fd, void *buffer, int  count)
{
    return -1;
}

inline int spiXfer(int fd, void *txBuf, void *rxBuf, int  count)
{
    return -1;
}