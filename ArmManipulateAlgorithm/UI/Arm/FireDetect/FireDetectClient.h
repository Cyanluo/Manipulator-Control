#ifndef WIDGET_H
#define WIDGET_H

#include <QObject>

#include "../networkHandle/MqttHandle.h"
#include "../Pointer/SharedPointer.h"
#include "../Pointer/SmartPointer.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QByteArray>
#include <QTimer>

#include "MainProcess.h"

using namespace cv;
using namespace FD;
using namespace GeneticAlgorithm;

class FireDetectClient : public QObject
{
    Q_OBJECT

    const int STATUSNUMBER = 3;

    enum STATUS
    {
        RunParamBufferFull,
        ActionExecute,
        AlgorithmDebugDataBufferFull,
        BUSY
    };

    SmartPointer<QByteArray> statusBuffer;
    SmartPointer<QByteArray> executeStatusBuffer;
    SmartPointer<MqttHandle> m_mqttClient;

    VideoCapture m_capture;
    QTimer* eventLoopTimer;

    MainProcess* GA_PSO;
    MatrixXd* limit;
    TransferMatrix* target;

    void actionExecute();
    void eventLoop(int eventid = -1);
    void run(int eventid);
    void mqttEventLoop();
    bool isSBFSet(int statusid);
    bool isESBFSet(int statusid);
    void setStatus(int statusid);
    void resetStatus(int statusid);
    void setEStatus(int statusid);
    void resetEStatus(int statusid);
    void initGAPSO();
public:
    FireDetectClient(QObject *parent = 0);

//slots
    void mqttEventDispose(QMQTT::Message message);
    void send_img();
    ~FireDetectClient();
};

#endif // WIDGET_H
