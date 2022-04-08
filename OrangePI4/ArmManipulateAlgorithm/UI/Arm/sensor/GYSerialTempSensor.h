#ifndef GYSERIALTEMPSENSOR_H
#define GYSERIALTEMPSENSOR_H

#include "TempSensor.h"
#include "../serial/AbstractSerial.h"

#define add 0xa4

struct ST_Tempture
{
    float to;
    float ta;
    float bo;
    float e;
};

class GYSerialTempSensor : public TempSensor<ST_Tempture, AbstractSerial>
{
public:
    explicit GYSerialTempSensor(SharedPointer<AbstractSerial> devcie);
    ST_Tempture getData();
    ~GYSerialTempSensor();
};


#endif // GYSERIALTEMPSENSOR_H
