#include "FireDetectClient.h"
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    FireDetectClient w;

    return a.exec();
}
