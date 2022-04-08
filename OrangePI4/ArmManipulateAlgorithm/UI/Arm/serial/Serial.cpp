#include "Serial.h"

Serial::Serial()
{

}

Serial* Serial::NewSerial(const char *device, const int baud)
{
    Serial* ret = new Serial();

    if(ret != nullptr)
    {
        ret->m_fd = ret->serialOpen(device, baud);

        if(ret->m_fd == -1)
        {
            delete ret;
            ret = nullptr;
        }
    }

    return ret;
}

int Serial::serialOpen (const char *device, const int baud)
{
    return ::serialOpen(device, baud);
}

int Serial::serialDataAvail ()
{
    return ::serialDataAvail(m_fd);
}

char Serial::serialGetchar ()
{
    return ::serialGetchar(m_fd);
}

void Serial::serialPuts (const int fd, const char *s)
{
    return ::serialPuts(m_fd, s);
}

void Serial::sendString(char* buf)
{
    return ::serialPuts(m_fd, buf);
}

Serial::~Serial()
{
    ::serialClose(m_fd);
}
