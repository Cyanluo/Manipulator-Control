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
#include <sstream>
#include "MainProcess.h"

using namespace cv;
using namespace FD;
using namespace GeneticAlgorithm;

#define TO_NUMBER(s, n) ( istringstream(s) >> n )

class FireDetectClient : public QObject
{
    Q_OBJECT

    const int STATUSNUMBER = 3;

    enum STATUS
    {
        ActionExecute,
        AlgorithmDebugDataBufferFull,
        RunParamBufferFull,
        ReceiveRunParams
    };

    SmartPointer<QByteArray> statusBuffer;
    SmartPointer<QByteArray> executeStatusBuffer;
    SmartPointer<MqttHandle> m_mqttClient;

    VideoCapture m_capture;
    QTimer* eventLoopTimer;

    MainProcess* GA_PSO;
    MatrixXd* limit;
    TransferMatrix* target;
    VectorXf* runParams;
    JMechArm<JOINTN, JOINTN>* arm;

    void actionExecute();
    void loadRunParams(QString& runParams);
    void deviceBusy(QString msg);

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
    void mqttCommendDispose(QMQTT::Message message);
    void mqttDataDispose(QMQTT::Message message);

    void send_img();
    ~FireDetectClient();
};

#endif // WIDGET_H
