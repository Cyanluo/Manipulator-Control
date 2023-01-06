#include "SmartPicker.h"
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    SmartPicker w;

    return a.exec();
}
