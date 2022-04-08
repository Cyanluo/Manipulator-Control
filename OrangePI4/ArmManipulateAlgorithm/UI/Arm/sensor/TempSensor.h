#ifndef TEMPSENSOR_H
#define TEMPSENSOR_H

#include "Sensor.h"
#include "../Pointer/SharedPointer.h"

using namespace FD;

template <typename T, typename D>
class TempSensor: public Sensor<T>
{
protected:
    SharedPointer<D> m_dev;
public:
    virtual void setDev(SharedPointer<D> devcie)
    {
        m_dev = devcie;
    }
};

#endif // TEMPSENSOR_H
