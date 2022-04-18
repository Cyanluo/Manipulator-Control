#include "FireDetectClient.h"
#include <QDebug>
#include <sstream>
#include <iomanip>

#define INT_TO_STRING(n) ( ((ostringstream&)(ostringstream() << n)).str() )

FireDetectClient::FireDetectClient(QObject *parent)
    : QObject(parent),algorithmType(0),mode(NormalRun)
{
    Login_config log_info =
    {
        QHostAddress("192.168.119.131"),
        "public",
        1883,
        "1234",
        "admin"
    };

    m_mqttClient = new MqttHandle(log_info);

    connect(m_mqttClient.get(), &MqttHandle::mqttReceiveCommand, this, &FireDetectClient::mqttCommendDispose);
    connect(m_mqttClient.get(), &MqttHandle::mqttReceiveData, this, &FireDetectClient::mqttDataDispose);

    executeStatusBuffer = new QByteArray(STATUSNUMBER, 0);
    statusBuffer = new QByteArray(STATUSNUMBER, 0);

    armSerialController = new ArmController("/dev/ttyS4", 115200);

    initGAPSO();

    eventLoopTimer = new QTimer(this);
    connect(eventLoopTimer, &QTimer::timeout, this, &FireDetectClient::mqttEventLoop);
    eventLoopTimer->start(200);

    m_capture.open(10);

    imageTimer = new QTimer(this);
    connect(imageTimer, &QTimer::timeout, this, &FireDetectClient::send_img);
    imageTimer->start(200);
}

void FireDetectClient::initGAPSO()
{
    GA_PSO = new MainProcess();
    limit = new MatrixXd(JOINTN+1, 2);
    target = new TransferMatrix();
    runParams = new VectorXf(JOINTN, 1);

    Matrix<float, JOINTN, 4> dh;
    dh << -PI/2,   0,   0,   0,
           PI/2,   0,  0,   PI/2,
            PI,     105,   0,  0,
            0,     98,  0,   0,
            0,     150,   0,  0;
//    dh << -PI/2,   0,   0,   0,
//           PI/2,   0,  0,   PI/2,
//            PI,     105,   0,  0,
//            0,     98,     0,   0,
//          -PI/2,     20,   40,  0,
//            PI/2,    110,   0,  0;

    TransferMatrix wf;

    arm = new DH_MechanicalArm<JOINTN, JOINTN>(dh, wf);

    (*limit) << -90, 90,
            -90, 90,
            -90, 90,
            -90, 90,
            0, 0,
            -9, 9; // 粒子群速度限制
//    (*limit) << -90, 90,
//            -90, 90,
//            -90, 90,
//            -90, 90,
//            -90, 90,
//            -90, 90,
//            -9, 9; // 粒子群速度限制

    GA_PSO->setDebug(true);

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

void FireDetectClient::mqttCommendDispose(QMQTT::Message message)
{
    if( message.topic().startsWith(m_mqttClient->topic("pc_command")) )
    {
        string str_cmd = message.payload().toStdString();

        int cmd = -1;
        TO_NUMBER(str_cmd, cmd);

        switch(cmd)
        {
            case ActionExecute:
            {
                actionExecute();
                break;
            }
            case ReachTarget:
            {
                reachTarget();
                break;
            }

            default: break;
        }
    }
}

//id;data
void FireDetectClient::mqttDataDispose(QMQTT::Message message)
{
    if( message.topic().startsWith(m_mqttClient->topic("pc_dataChannel1")) )
    {
        string data = message.payload().toStdString();
        QString qdata = QString::fromStdString(data);

        QStringList qdataList = qdata.split(';', QString::SkipEmptyParts);
        QString str_dtype = qdataList.at(0);

        int dtype = -1;
        TO_NUMBER(str_dtype.toStdString(), dtype);

        cout << ReceiveTarget << endl;
        switch(dtype)
        {
            case ReceiveRunParams:
            {
                loadRunParams( const_cast<QString&>(qdataList.at(1)) );
                break;
            }
            case ReceiveTarget:
            {
                loadTarget( const_cast<QString&>(qdataList.at(1)) );
                break;
            }
            case ReceivePoints:
            {
                loadPoints( const_cast<QString&>(qdataList.at(1)) );
                break;
            }
            case SettingData:
            {
                setting( const_cast<QString&>(qdataList.at(1)) );
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

void FireDetectClient::runGAPSO(TransferMatrix* ltarget, int type)
{
    switch(type)
    {
        case 0:
        {
            GA_PSO->setPSO(
                0.2, // w 惯性权重
                1.8, // c1 种群内权重
                2	 // c2 种群间权重
            );

             GA_PSO->run(
                5,	// 种群数量
                25, // 种群大小
                JOINTN, // 染色体长度
                *limit, // 每个基因的限制
                100, // 最大迭代次数
                99, // 停止迭代适应度
                10, // 每次迭代保留多少个上一代的高适应度个体
                1.1, // 变异调整参数
                *ltarget // 寻优的目标
            );
            cout << "GA_PSO" << endl;
            break;
        }
        case 1:
        {
            GA_PSO->disablePSO();

            GA_PSO->run(
                    1,	// 种群数量
                    125, // 种群大小
                    JOINTN, // 染色体长度
                    *limit, // 每个基因的限制
                    100, // 最大迭代次数
                    99.9, // 停止迭代适应度
                    25, // 每次迭代保留多少个上一代的高适应度个体
                    1.1, // 变异调整参数
                    *ltarget // 寻优的目标
                );
            cout << "GA" << endl;
            break;

        }
        case 2:
        {
            GA_PSO->setPSO(
                0.2, // w 惯性权重
                1.8, // c1 种群内权重
                2	 // c2 种群间权重
            );

             GA_PSO->run(
                5,	// 种群数量
                25, // 种群大小
                JOINTN, // 染色体长度
                *limit, // 每个基因的限制
                100, // 最大迭代次数
                99.99, // 停止迭代适应度
                25, // 每次迭代保留多少个上一代的高适应度个体
                1.1, // 变异调整参数
                *ltarget // 寻优的目标
            );
            cout << "PSO" << endl;
            break;
        }

        default: break;
    }
}

void FireDetectClient::runArm()
{
    string plotData = GA_PSO->getPlotData();

    string head = "plotData:";
    plotData = head + plotData;

    m_mqttClient->Publish_data(m_mqttClient->topic("armControllerNode_dataChannel1"),
                               QString::fromStdString(plotData));

    if(mode == NormalRun)
    {
        VectorXd ids(JOINTN, 1), times(JOINTN, 1);
        VectorXf rp(JOINTN, 1);
        ids << 0, 1, 2, 3, 4;
        times << 3000, 3000, 3000, 3000, 3000;
        for(int i=0; i<JOINTN; i++)
        {
            rp(i) = ( (*runParams)(i)*(180/PI) );
            cout << rp(i) << " ";
        }
        cout << endl;

        armSerialController->openClaw();
        armSerialController->actionExe(ids, rp, times);
        armSerialController->closeClaw();
        armSerialController->resetPostion();

        armSerialController->run(1);
    }
}

void FireDetectClient::run(int eventid)
{
    switch (eventid)
    {
        case ActionExecute:
        {
            if( isSBFSet(RunParamBufferFull) )
            {
                resetStatus(ActionExecute); // cup ack
                setEStatus(ActionExecute); // runing

//                (*target) = (*arm).forward(*runParams);
//                runGAPSO(target, this->algorithmType);

                runArm();

                resetStatus(RunParamBufferFull);
                resetEStatus(ActionExecute); // finish
            }

            break;
        }
        case ReachTarget:
        {
            if( isSBFSet(TargetBufferFull) )
            {
                resetStatus(ReachTarget); // cup ack
                setEStatus(ReachTarget); // runing

                if(mode == NormalRun)
                {
                    runGAPSO(target, this->algorithmType);
                    (*runParams) = GA_PSO->getResultRunParams();
                    runArm();
                }
                else if(mode == TestAlgorithm)
                {
                    for(int i=0; i<3; i++)
                    {
                        runGAPSO(target, i);
                        (*runParams) = GA_PSO->getResultRunParams();
                        runArm();
                    }
                }
                else
                {

                }

                resetStatus(TargetBufferFull);
                resetEStatus(ReachTarget); // finish
            }
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
        deviceBusy("BUSY: actionExecute");
    }
}

void FireDetectClient::reachTarget()
{
    if( (!(isESBFSet(ReachTarget))) && (!(isSBFSet(ReachTarget))) )
    {
        setStatus(ReachTarget);
    }
    else
    {
        deviceBusy("BUSY: reachTarget");
    }
}

void FireDetectClient::loadRunParams(QString& runParams)
{
    if(!isSBFSet(RunParamBufferFull))
    {               
        setStatus(RunParamBufferFull);

        QStringList qdataList = runParams.split(',', QString::SkipEmptyParts);

        double temp = 0;

        for(int i=0; i<JOINTN; i++)
        {
            TO_NUMBER(const_cast<QString&>(qdataList.at(i)).toStdString(), temp);
            (*(this->runParams))[i] = RADIAN(temp);
        }
    }
}

void FireDetectClient::loadTarget(QString& target)
{
    if(!isSBFSet(TargetBufferFull))
    {
        setStatus(TargetBufferFull);

        QStringList qdataList = target.split(',', QString::SkipEmptyParts);

        double temp = 0;

        cout << qdataList.length() << endl;
        cout << target.toStdString() << endl;

        for(int i=0; i<4; i++)
        {
            for(int j=0; j<4; j++)
            {
                cout << i*4+j << ":" << qdataList.at(i*4+j).toStdString() << endl;
                TO_NUMBER(const_cast<QString&>(qdataList.at(i*4+j)).toStdString(), temp);
                (*(this->target))(i, j) = temp;
            }
        }
    }
}

//points:"1,2,3,4"->(1,2)(3,4)
void FireDetectClient::loadPoints(QString& points)
{
    if(!isSBFSet(PointsBufferFull))
    {
        setStatus(PointsBufferFull);

        QStringList qdataList = points.split(',', QString::SkipEmptyParts);

        double temp = 0;

        TO_NUMBER(const_cast<QString&>(qdataList.at(0)).toStdString(), temp);
        cleft.x = temp;
        TO_NUMBER(const_cast<QString&>(qdataList.at(1)).toStdString(), temp);
        cleft.y = temp;
        TO_NUMBER(const_cast<QString&>(qdataList.at(2)).toStdString(), temp);
        cright.x = temp;
        TO_NUMBER(const_cast<QString&>(qdataList.at(3)).toStdString(), temp);
        cright.y = temp;

        Point3d tri_xyz;
        tri_xyz = m_capture.triangulation(cleft, cright);

        cout << tri_xyz << endl;

        string data = "pointXYZ:";
        data += INT_TO_STRING(tri_xyz.x);
        data += ",";
        data += INT_TO_STRING(tri_xyz.y);
        data += ",";
        data += INT_TO_STRING(tri_xyz.z);

        m_mqttClient->Publish_data(m_mqttClient->topic("armControllerNode_dataChannel1"),
                                   QString::fromStdString(data));

        resetStatus(PointsBufferFull);
    }
}

// setType:data1,data2
void FireDetectClient::setting(QString& settingData)
{
    if(!isSBFSet(SettingDataBufferFull))
    {
        setStatus(SettingDataBufferFull);

        QStringList qdataList = settingData.split(':', QString::SkipEmptyParts);
        QString setType = qdataList.at(0);

        if(setType.startsWith("algorithmType"))
        {
            double temp = 0;
            TO_NUMBER(const_cast<QString&>(qdataList.at(1)).toStdString(), temp);
            cout << "temp:" << temp << endl;
            this->algorithmType = temp;

            deviceBusy("Setting:SUCCESS:algorithmType");
        }
        else if(setType.startsWith("mode"))
        {
            double temp = 0;
            TO_NUMBER(const_cast<QString&>(qdataList.at(1)).toStdString(), temp);

            this->mode = temp;

            deviceBusy("Setting:SUCCESS:mode");
        }
        else
        {
            deviceBusy("Setting:ERROR:wrong setType!!!");
        }

        resetStatus(SettingDataBufferFull);
    }
}

void FireDetectClient::deviceBusy(QString msg)
{
    m_mqttClient->Publish_data(m_mqttClient->topic("busy"), msg);
}

void FireDetectClient::send_img()
{
    Mat srcL, srcR;
    m_capture.getCorrectedImg(srcL, srcR);

#ifdef SGBM
    static int times = 0;
    if(times >= 5)
    {
        times = 0;
        Mat deepImg;
        xyz = m_capture.stereoMatch(srcL, srcR, deepImg);
        m_mqttClient->Publich_img(deepImg, "deepMap");
    }
    else
    {
        times++;
    }
#endif

    Mat merge;
    merge = m_capture.mergingDisplay(srcL, srcR);

    m_mqttClient->Publich_img(merge, "LRImage");

}

FireDetectClient::~FireDetectClient()
{
    delete eventLoopTimer;
    delete GA_PSO;
    delete limit;
    delete target;
    delete runParams;
    delete armSerialController;
    delete imageTimer;
}
