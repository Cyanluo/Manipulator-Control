#include "armController.h"

#include <iomanip>
#include <sstream>
#include <iostream>
#include "Chromosome.h"
using namespace std;

#define INT_TO_STRING(n, fill) ( ((ostringstream&)(ostringstream() << setw(fill) << setfill('0') << n)).str() )
#define PWM(n) (1500 + (n*(2000.0/270.0)))

ArmController::ArmController(QObject *parent):QObject(parent)
{

}

ArmController::ArmController(const char *device, int buad, QObject *parent):QObject(parent)
{
    m_dev = Serial::NewSerial(device, buad);

    busy = false;
    times = 0;
    currentTimes = 0;

    actionTimer = new QTimer(this);
    connect(actionTimer, &QTimer::timeout, this, &ArmController::runOnce);
}

ArmController::~ArmController()
{
    delete actionTimer;
}

void ArmController::actionExe(VectorXd& ids, VectorXf& runParams, VectorXd& time)
{
    string da = "{";

    for(int i=0; i<runParams.rows(); i++)
    {
        if(ids(i) >= 0)
        {
            da += "#";
            da += INT_TO_STRING(ids(i), 3);
            da += "P";
            da += INT_TO_STRING(int(PWM(runParams(i))), 4);
            da += "T";
            da += INT_TO_STRING(time(i), 4);
            da += "!";
        }
    }
    da += "}";
    cout << da << endl;

    cmdQueueBak.enqueue(da);
    actionTimeQueueBak.enqueue(time.maxCoeff());
    //m_dev->sendString(const_cast<char*>(da.c_str()));
}

void ArmController::runOnce()
{
    if(cmdQueue.length() > 0)
    {
        m_dev->sendString(const_cast<char*>(cmdQueue.dequeue().c_str()));

        int atime = actionTimeQueue.dequeue();

        if(cmdQueue.length() > 0)
        {
            actionTimer->start(atime);
        }
        else
        {
            currentTimes++;

            if(currentTimes < times)
            {
                cmdQueue = cmdQueueBak;
                actionTimeQueue = actionTimeQueueBak;

                actionTimer->start(atime);
            }
            else
            {
                actionTimer->stop();

                actionTimeQueueBak.clear();
                cmdQueueBak.clear();

                busy = false;
            }
        }
    }
}

void ArmController::run(int times)
{   
    if(!busy)
    {
        busy = true;
        currentTimes = 0;
        this->times = times;
        cmdQueue = cmdQueueBak;
        actionTimeQueue = actionTimeQueueBak;

        runOnce();
    }
}

void ArmController::openClaw()
{
    VectorXf runParams(1);
    VectorXd ids(1), time(1);

    runParams << -80;
    ids << 5;
    time << 1000;

    actionExe(ids, runParams, time);
}

void ArmController::closeClaw()
{
    VectorXf runParams(1);
    VectorXd ids(1), time(1);

    runParams << 0;
    ids << 5;
    time << 1000;

    actionExe(ids, runParams, time);
}

void ArmController::resetPostion()
{
    VectorXf runParams(JOINTN);
    VectorXd ids(JOINTN), time(JOINTN);

    for(int i=0; i<JOINTN; i++)
    {
        runParams(i) = 0;
        ids(i) = i;
        time(i) = 1000;
    }

    actionExe(ids, runParams, time);

    openClaw();
}
