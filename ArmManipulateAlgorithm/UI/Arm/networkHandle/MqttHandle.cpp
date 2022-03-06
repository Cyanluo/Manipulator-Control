#include "MqttHandle.h"
#include <QDebug>
#include <QTimer>

MqttHandle::MqttHandle(Login_config& login_info, QObject *parent): QObject(parent)
{     
    this->login_info = login_info;

    init_topics();

    connect_mqtt();
}

void MqttHandle::init_topics()
{
    topics["pc_command"] = "/pc/command/1";
    topics["client_img"] = "/client/image/" + login_info.deviceId;
    topics["armControllerNode_dataChannel1"] = "/armControllerNode/dataChannel1/" + login_info.deviceId;
}

QString MqttHandle::topic(QString tp)
{
    return topics[tp];
}

void MqttHandle::connect_mqtt()
{
    client = new QMQTT::Client(); // 初始化QMQTT客户指针

    connect(client, &QMQTT::Client::received, this, &MqttHandle::onMQTT_Received);
    connect(client, &QMQTT::Client::error, this, &MqttHandle::onMQTT_Error);
    connect(client, &QMQTT::Client::connected, this, &MqttHandle::QMQTT_connected);

    client->setKeepAlive(120); // 心跳
    client->setHost(login_info.host); // 设置 EMQ 代理服务器
    client->setPort(login_info.port); // 设置 EMQ 代理服务器端口
    client->setClientId(login_info.deviceId); // 设备 ID
    client->setUsername(login_info.productId); // 产品 ID
    client->setPassword(login_info.password);
    client->cleanSession();
    //client->setVersion(QMQTT::MQTTVersion::V); // 设置mqtt版本

    client->connectToHost(); // 连接 EMQ 代理服务器

    QTimer::singleShot(1000, this, [=](){
        client->subscribe(topics["pc_command"], 1);
    });
}

void MqttHandle::onMQTT_Received( QMQTT::Message message)
{
    emit mqttReceive(message);
}

void MqttHandle::onMQTT_Error(QMQTT::ClientError error)
{
    qDebug() << error << endl;
}

void MqttHandle::QMQTT_connected()
{
    qDebug() << "ok" << endl;
}

void MqttHandle::Publish_data(QString topic, QString data)
{
    if (client->publish(QMQTT::Message(0, topic, QByteArray(data.toUtf8()))))
    {
        qDebug() << "public error !!!" << endl;
    }
}

void MqttHandle::Publich_img(Mat& image)
{
    std::vector<uchar> img = image_encode(image);
    char arr[img.size()];
    std::copy(img.begin(), img.end(), arr);
    // std::cout << img.size() << std::endl;
    if (client->publish(QMQTT::Message(0, topics["client_img"], QByteArray(arr, img.size()))))
    {
        qDebug() << "public error !!!" << endl;
    }
}

std::vector<uchar> MqttHandle::image_encode(Mat& src)
{
    std::vector<uchar> buff;
    std::vector<int> params;

    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(50);
    cv::imencode(".jpg", src, buff, params);

    return buff;
}

