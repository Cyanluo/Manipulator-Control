#include "Exception.h"
#include <cstdlib>
#include <cstring>

using namespace std;
using namespace FD;

void Exception::init(const char* message, const char* file, int line)
{
    m_message = message ? strdup(message): NULL;

    if(file != NULL)
    {
        char ret[16] = {0};

        //itoa(line, ret, 10);

        m_location = (char *)malloc(strlen(file) + strlen(ret) + 2);

        if(m_location != NULL)
        {
            strcpy(m_location, file);
            strcat(m_location, ":");
            strcat(m_location, ret);
        }
    }
    else
    {
        m_location = NULL;
    }
}

Exception::Exception(const char* message, const char* file, int line)
{
    init(message, file, line);
}

Exception::Exception(const char* message)
{
    init(message, NULL, 0);
}

Exception::Exception(const char* file, int line)
{
    init(NULL, file, line);
}

Exception::Exception(const Exception& e)
{
    m_message = strdup(e.m_message);
    m_location = strdup(e.m_location);
}

Exception& Exception::operator =(const Exception& e)
{
    if(this != &e)
    {
        free(m_message);
        free(m_location);

        m_message = strdup(e.m_message);
        m_location = strdup(e.m_message);
    }

    return *this;
}

const char* Exception::message() const
{
    return m_message;
}

const char* Exception::location() const
{
    return m_location;
}

Exception::~Exception()
{
    free(m_message);
    free(m_location);
}

