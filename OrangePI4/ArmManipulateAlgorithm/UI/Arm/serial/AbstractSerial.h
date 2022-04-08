#ifndef ABSTRACTSERIAL_H
#define ABSTRACTSERIAL_H

class AbstractSerial
{
protected:
    AbstractSerial(const AbstractSerial&);
    AbstractSerial& operator = (const AbstractSerial&);
public:
    AbstractSerial(){}
    virtual int serialDataAvail () = 0;
    virtual char serialGetchar () = 0;
    virtual void serialPuts (const int fd, const char *s) = 0;
    virtual void sendString(char* buf){}
    virtual ~AbstractSerial(){}
};


#endif // ABSTRACTSERIAL_H
