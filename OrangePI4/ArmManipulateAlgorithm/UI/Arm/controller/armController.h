#ifndef ARMCONTROLLER_H_
#define ARMCONTROLLER_H_

#include "../Pointer/SharedPointer.h"
#include "../serial/AbstractSerial.h"
#include "../serial/Serial.h"
#include "Dense"
#include "Core"
#include <QTimer>
#include <QObject>
#include <QQueue>
#include <string>

using namespace FD;
using namespace Eigen;


class ArmController:public QObject
{
    Q_OBJECT

    SharedPointer<AbstractSerial> m_dev;
    QTimer* actionTimer;
    QQueue<std::string> cmdQueue;
    QQueue<int> actionTimeQueue;
    QQueue<std::string> cmdQueueBak;
    QQueue<int> actionTimeQueueBak;
    bool busy;
    int currentTimes;
    int times;

    void runOnce();

public:
    ArmController(QObject *parent = 0);
    ArmController(const char *device, int buad, QObject *parent = 0);
    void actionExe(VectorXd& ids, VectorXf& runParams, VectorXd& time);
    void run(int times=1);
    void openClaw();
    void closeClaw();
    void resetPostion();
    ~ArmController();
};

#endif
