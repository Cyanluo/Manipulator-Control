#include "PinController.h"

PinController::PinController(): m_pin(-1)
{
    ::wiringPiSetup();
}

PinController::PinController(int pin): m_pin(pin)
{
    ::wiringPiSetup();
}

void PinController::pinMode(int pin, int mode)
{
    ::pinMode(pin, mode);
}

int  PinController::digitalRead(int pin)
{
    ::digitalRead(pin);
}

void PinController::digitalWrite(int pin, int value)
{
    ::digitalWrite(pin, value);
}

int PinController::pinMode(int mode)
{
    int ret = -1;

    ::pinMode(m_pin, mode);

    ret = m_pin == -1 ? -1 : 1;

    return ret;
}

int  PinController::digitalRead()
{
    int ret = -1;

    ret = m_pin == -1 ? -1 : ::digitalRead(m_pin);

    return ret;
}

int PinController::digitalWrite(int value)
{
    int ret = -1;

    ::digitalWrite(m_pin, value);

    ret = m_pin == -1 ? -1 : 1;

    return ret;
}

void PinController::set_pin(int pin)
{
    m_pin = pin;
}
