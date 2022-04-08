#ifndef SENSOR_H
#define SENSOR_H

namespace FD {
template <typename T>
class Sensor
{
protected:
    Sensor(const Sensor&);
    Sensor& operator = (const Sensor&);
public:
    Sensor(){}
    virtual T getData() = 0;
    virtual ~Sensor(){}
};
}



#endif // SENSOR_H
