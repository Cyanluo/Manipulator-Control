#ifndef SHAREDPOINTER_H
#define SHAREDPOINTER_H

#include "Pointer.h"
#include "cstdlib"
#include "Exception.h"

namespace FD
{

template < typename T >
class SharedPointer: public Pointer<T>
{
protected:
    int* m_ref;

    void assign(const SharedPointer<T>& obj)
    {
        this->m_ref = obj.m_ref;
        this->m_pointer = obj.m_pointer;

        if(this->m_ref)
        {
            (*this->m_ref)++;
        }
    }
public:
    SharedPointer(T* p=NULL): m_ref(NULL)
    {
        if(p)
        {
            this->m_ref = reinterpret_cast<int*>(std::malloc(sizeof(int)));

            if(this->m_ref)
            {
                this->m_pointer = p;
                *this->m_ref = 1;
            }
            else
            {
                THROW_EXCEPTION(NoEnoughMemonyException, "no memory to create SharedPointer");
            }
        }
    }

    SharedPointer(const SharedPointer<T>& obj): Pointer<T>(NULL)
    {
        assign(obj);
    }

    SharedPointer<T>& operator =(const SharedPointer<T>& obj)
    {
        if(&obj != this)
        {
            clear();
            assign(obj);
        }

        return *this;
    }

    void clear()
    {
        int* ref = this->m_ref;
        T* p = this->m_pointer;

        this->m_ref = NULL;
        this->m_pointer = NULL;
        if(ref)
        {
            (*ref)--;

            if(*ref == 0)
            {
                delete p;
                std::free(ref);
            }
        }
    }

    ~SharedPointer()
    {
        clear();
    }
};

template < typename T >
bool operator ==(const SharedPointer<T>& l, const SharedPointer<T>& r)
{
    return (l.get() == r.get());
}

template < typename T >
bool operator !=(const SharedPointer<T>& l, const SharedPointer<T>& r)
{
    return !(l == r);
}

}

#endif // SHAREDPOINTER_H
