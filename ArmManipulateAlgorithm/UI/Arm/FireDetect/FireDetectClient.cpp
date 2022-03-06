#include "FireDetectClient.h"
#include <QDebug>

FireDetectClient::FireDetectClient(QObject *parent)
    : QObject(parent)
{
    Login_config log_info =
    {
        QHostAddress("127.0.0.1"),
        "public",
        1883,
        "1234",
        "admin"
    };

    m_mqttClient = new MqttHandle(log_info);

    connect(m_mqttClient.get(), &MqttHandle::mqttReceive, this, &FireDetectClient::mqttEventDispose);

    executeStatusBuffer = new QByteArray(STATUSNUMBER, 0);
    statusBuffer = new QByteArray(STATUSNUMBER, 0);

    initGAPSO();

    eventLoopTimer = new QTimer(this);
    connect(eventLoopTimer, &QTimer::timeout, this, &FireDetectClient::mqttEventLoop);
    eventLoopTimer->start(200);

//    m_capture.open(10);

//    QTimer *timer = new QTimer(this);
//    connect(timer, &QTimer::timeout, this, &FireDetectClient::send_img);
//    timer->start(500);
}

void FireDetectClient::initGAPSO()
{
    GA_PSO = new MainProcess();
    limit = new MatrixXd(JOINTN+1, 2);
    target = new TransferMatrix();

    Matrix<float, JOINTN, 4> dh;
    dh << -PI/2,   0,   2,   0,
            PI/2,   0,  0, PI/2,
            0,     10,   0,  0,
            0,     20,  0,   0,
            0,     30,   0,  0;

    TransferMatrix wf;

    DH_MechanicalArm<JOINTN, JOINTN> arm(dh, wf);

    VectorXf runParams(JOINTN, 1);
    runParams << RADIAN(-20),RADIAN(50),RADIAN(-20),RADIAN(-80),RADIAN(10);
    (*target) = arm.forward(runParams);

    (*limit) << -90, 90,
            -90, 90,
            -90, 90,
            -90, 10,
            -90, 90,
            -9, 9; // 粒子群速度限制

    GA_PSO->setDebug(false);

    GA_PSO->setPSO(
            0.2, // w 惯性权重
            1.8, // c1 种群内权重
            2	 // c2 种群间权重
        );
}

bool FireDetectClient::isSBFSet(int statusid)
{
    return (*statusBuffer).at(statusid);
}

bool FireDetectClient::isESBFSet(int statusid)
{
    return (*executeStatusBuffer).at(statusid);
}

void FireDetectClient::setStatus(int statusid)
{
    (*statusBuffer)[statusid] = 1;
}

void FireDetectClient::resetStatus(int statusid)
{
    (*statusBuffer)[statusid] = 0;
}

void FireDetectClient::setEStatus(int statusid)
{
    (*executeStatusBuffer)[statusid] = 1;
}

void FireDetectClient::resetEStatus(int statusid)
{
    (*executeStatusBuffer)[statusid] = 0;
}

void FireDetectClient::mqttEventDispose(QMQTT::Message message)
{
    if( message.topic().startsWith(m_mqttClient->topic("pc_command")) )
    {
        QByteArray data = message.payload();

        switch(data.at(0))
        {
            case ActionExecute:
            {
                actionExecute();
                break;
            }

            default: break;
        }
    }
}

void FireDetectClient::mqttEventLoop()
{
    int id = -1;

    for(int i=0; i<STATUSNUMBER; i++)
    {
        if( isSBFSet(i) )
        {
            id = i;
            break;
        }
    }

    if(id >= 0)
    {
        run(id);
    }
}

void FireDetectClient::run(int eventid)
{
    switch (eventid)
    {
        case ActionExecute:
        {
            if( !isSBFSet(RunParamBufferFull) )
            {
                resetStatus(ActionExecute); // cup ack
                setEStatus(ActionExecute); // runing

                GA_PSO->run(
                        5,	// 种群数量
                        25, // 种群大小
                        JOINTN, // 染色体长度
                        *limit, // 每个基因的限制
                        40, // 最大迭代次数
                        20, // 停止迭代适应度
                        18, // 每次迭代保留多少个上一代的高适应度个体
                        1.1, // 变异调整参数
                        *target // 寻优的目标
                    );

                string plotData = GA_PSO->getPlotData();

                m_mqttClient->Publish_data(m_mqttClient->topic("armControllerNode_dataChannel1"),
                                           QString::fromStdString(plotData));

                resetStatus(RunParamBufferFull);
                resetEStatus(ActionExecute); // finish
            }

            break;
        }
        case BUSY:
        {
            break;
        }

        default: break;
    }
}

void FireDetectClient::actionExecute()
{
    if( (!(isESBFSet(ActionExecute))) && (!(isSBFSet(ActionExecute))) )
    {
        setStatus(ActionExecute);
    }
    else
    {
        setStatus(BUSY);
    }
}

void FireDetectClient::send_img()
{
    Mat img;

    m_capture >> img;

    m_mqttClient->Publich_img(img);

}

FireDetectClient::~FireDetectClient()
{
    delete eventLoopTimer;
    delete GA_PSO;
    delete limit;
    delete target;
}
