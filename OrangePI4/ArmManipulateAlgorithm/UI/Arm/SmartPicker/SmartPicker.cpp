#include "SmartPicker.h"
#include <QDebug>
#include <sstream>
#include <iomanip>

#define INT_TO_STRING(n) ( ((ostringstream&)(ostringstream() << n)).str() )

SmartPicker::SmartPicker(QObject *parent)
    : QObject(parent),algorithmType(0),mode(NormalRun)
{
    Login_config log_info =
    {
        QHostAddress("192.168.0.106"),
        "public",
        1883,
        "1234",
        "admin"
    };

    m_mqttClient = new MqttHandle(log_info);

    connect(m_mqttClient.get(), &MqttHandle::mqttReceiveCommand, this, &SmartPicker::mqttCommendDispose);
    connect(m_mqttClient.get(), &MqttHandle::mqttReceiveData, this, &SmartPicker::mqttDataDispose);

    executeStatusBuffer = new QByteArray(STATUSNUMBER, 0);
    statusBuffer = new QByteArray(STATUSNUMBER, 0);

    armSerialController = new ArmController("/dev/ttyS4", 115200);

    initGAPSO();

    eventLoopTimer = new QTimer(this);
    connect(eventLoopTimer, &QTimer::timeout, this, &SmartPicker::mqttEventLoop);
    eventLoopTimer->start(200);

    m_capture.open(10);

    imageTimer = new QTimer(this);
    connect(imageTimer, &QTimer::timeout, this, &SmartPicker::send_img);
    imageTimer->start(200);
}

void SmartPicker::initGAPSO()
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

    GA_PSO->setDebug(false);

    GA_PSO->setPSO(
            0.2, // w 惯性权重
            1.8, // c1 种群内权重
            2	 // c2 种群间权重
        );
}

bool SmartPicker::isSBFSet(int statusid)
{
    return (*statusBuffer).at(statusid);
}

bool SmartPicker::isESBFSet(int statusid)
{
    return (*executeStatusBuffer).at(statusid);
}

void SmartPicker::setStatus(int statusid)
{
    (*statusBuffer)[statusid] = 1;
}

void SmartPicker::resetStatus(int statusid)
{
    (*statusBuffer)[statusid] = 0;
}

void SmartPicker::setEStatus(int statusid)
{
    (*executeStatusBuffer)[statusid] = 1;
}

void SmartPicker::resetEStatus(int statusid)
{
    (*executeStatusBuffer)[statusid] = 0;
}

void SmartPicker::mqttCommendDispose(QMQTT::Message message)
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
void SmartPicker::mqttDataDispose(QMQTT::Message message)
{
    if( message.topic().startsWith(m_mqttClient->topic("pc_dataChannel1")) )
    {
        string data = message.payload().toStdString();
        json j_data = json::parse(data);

        cout << std::setw(4) << j_data << "\n\n";

        int dtype = -1;
        dtype = j_data["id"];

        switch(dtype)
        {
            case ReceiveRunParams:
            {
                loadRunParams(j_data);
                break;
            }
            case ReceiveTarget:
            {
                loadTarget(j_data);
                break;
            }
            case ReceivePoints:
            {
                loadPoints(j_data);
                break;
            }
            case SettingData:
            {
                setting(j_data);
                break;
            }

            default: break;
        }
    }
}

void SmartPicker::mqttEventLoop()
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

void SmartPicker::runGAPSO(TransferMatrix* ltarget, int type)
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
                50, // 最大迭代次数
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

void SmartPicker::runArm()
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

void SmartPicker::run(int eventid)
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

void SmartPicker::actionExecute()
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

void SmartPicker::reachTarget()
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

void SmartPicker::loadRunParams(json& runParams)
{
    if(!isSBFSet(RunParamBufferFull))
    {               
        setStatus(RunParamBufferFull);

        vector<float> j_runParams = runParams["runParams"];

        for(int i=0; i<JOINTN; i++)
        {
            (*(this->runParams))[i] = RADIAN(j_runParams[i]);
        }
    }
}

void SmartPicker::loadTarget(json& target)
{
    if(!isSBFSet(TargetBufferFull))
    {
        setStatus(TargetBufferFull);

        vector<vector<float>> tg = target["target"];

        for(int i=0; i<4; i++)
        {
            for(int j=0; j<4; j++)
            {
                (*(this->target))(i, j) = tg[i][j];
            }
        }
    }
}

void SmartPicker::loadPoints(json& points)
{
    if(!isSBFSet(PointsBufferFull))
    {
        setStatus(PointsBufferFull);

        vector<vector<double>> j_points = points["points"];

        double temp = 0;

        cleft.x = j_points[0][0];
        cleft.y = j_points[0][1];
        cright.x = j_points[1][0];
        cright.y = j_points[1][1];

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

void SmartPicker::setting(json& settingData)
{
    if(!isSBFSet(SettingDataBufferFull))
    {
        setStatus(SettingDataBufferFull);

        string std_setType = settingData["setType"];
        QString setType = QString::fromStdString(std_setType);

        if(setType.startsWith("algorithmType"))
        {
            this->algorithmType = settingData["setData"];

            deviceBusy("Setting:SUCCESS:algorithmType");
        }
        else if(setType.startsWith("mode"))
        {
            this->mode = settingData["setData"];

            deviceBusy("Setting:SUCCESS:mode");
        }
        else
        {
            deviceBusy("Setting:ERROR:wrong setType!!!");
        }

        resetStatus(SettingDataBufferFull);
    }
}

void SmartPicker::deviceBusy(QString msg)
{
    m_mqttClient->Publish_data(m_mqttClient->topic("busy"), msg);
}

void SmartPicker::send_img()
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

SmartPicker::~SmartPicker()
{
    delete eventLoopTimer;
    delete GA_PSO;
    delete limit;
    delete target;
    delete runParams;
    delete armSerialController;
    delete imageTimer;
}
