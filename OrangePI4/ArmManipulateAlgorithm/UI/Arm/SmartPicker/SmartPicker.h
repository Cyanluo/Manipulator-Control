#ifndef WIDGET_H
#define WIDGET_H

#include <QObject>

#include "../networkHandle/MqttHandle.h"
#include "../Pointer/SharedPointer.h"
#include "../Pointer/SmartPointer.h"
#include "../binocularCamera/binocularCamera.h"

#include "nlohmann/json.hpp"
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QByteArray>
#include <QTimer>
#include <sstream>
#include "MainProcess.h"

#include "../controller/armController.h"

using namespace cv;
using namespace FD;
using namespace GeneticAlgorithm;
using json = nlohmann::json;

#define TO_NUMBER(s, n) ( istringstream(s) >> n )
#define R_TO_ANGEL(r) ()

class SmartPicker : public QObject
{
    Q_OBJECT

    enum STATUS
    {
        NOP,
        ActionExecute,
        ReachTarget,
        AlgorithmDebugDataBufferFull,
        RunParamBufferFull,
        TargetBufferFull,
        PointsBufferFull,
        SettingDataBufferFull,
        STATUSNUMBER,
        ReceiveRunParams,
        ReceiveTarget,
        ReceivePoints,
        SettingData
    };

    enum MODE
    {
        NOP_M,
        TestAlgorithm,
        NormalRun
    };

    SmartPointer<QByteArray> statusBuffer;
    SmartPointer<QByteArray> executeStatusBuffer;
    SmartPointer<MqttHandle> m_mqttClient;

    binocularCamera m_capture;
    QTimer* eventLoopTimer;
    QTimer* imageTimer;

    MainProcess* GA_PSO;
    MatrixXd* limit;
    TransferMatrix* target;
    VectorXf* runParams;
    Point cleft;
    Point cright;
    int algorithmType;
    int mode;
    JMechArm<JOINTN, JOINTN>* arm;
    ArmController* armSerialController;

    Mat xyz;

    void actionExecute();
    void reachTarget();

    void loadRunParams(json& runParams);
    void deviceBusy(QString msg);
    void loadTarget(json& target);
    void loadPoints(json& points);
    void setting(json& settingData);

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
    void runGAPSO(TransferMatrix* ltarget, int type=0); // 0:ga-pso 1:ga 2:pso
    void runArm();
public:
    SmartPicker(QObject *parent = 0);

//slots
    void mqttCommendDispose(QMQTT::Message message);
    void mqttDataDispose(QMQTT::Message message);

    void send_img();
    ~SmartPicker();
};

#endif // WIDGET_H
