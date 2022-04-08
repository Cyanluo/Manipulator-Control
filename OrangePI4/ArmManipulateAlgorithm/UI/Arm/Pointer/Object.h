#ifndef OBJECT_H
#define OBJECT_H

namespace FD
{

class Object
{
public:
    void* operator new(unsigned long size)throw();
    void operator delete(void* p);
    void* operator new[](unsigned long size)throw();
    void operator delete[](void* p);
    bool operator ==(const Object& obj);
    bool operator !=(const Object& obj);
    virtual ~Object() = 0;
};

}


#endif // OBJECT_H
