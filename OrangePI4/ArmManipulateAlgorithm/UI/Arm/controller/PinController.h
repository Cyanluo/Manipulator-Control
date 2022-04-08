#ifndef PINCONTROLLER_H
#define PINCONTROLLER_H

#include <wiringPi.h>

class PinController
{
protected:
    int m_pin;
public:
    PinController();
    PinController(int pin);
    void pinMode(int pin, int mode);
    int pinMode(int mode);
    int  digitalRead(int pin);
    int  digitalRead();
    void digitalWrite(int pin, int value);
    int digitalWrite(int value);
    void set_pin(int pin);
};

#endif // PINCONTROLLER_H
