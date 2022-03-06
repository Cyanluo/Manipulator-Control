#ifndef MQTTHANDLE_H
#define MQTTHANDLE_H

#include "QObject"

#include "../qmqtt/qmqtt.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QMap>

using namespace cv;

struct Login_config
{
    QHostAddress host; // 代理服务器 IP
    QByteArray password; // 设备名称
    quint16 port; // 代理服务器端口
    QString deviceId; // 设备 ID
    QString productId;   // 产品 ID
};

class MqttHandle: public QObject
{
    Q_OBJECT
protected:
    QMap<QString, QString> topics;

    QMQTT::Client* client;
    Login_config login_info;
    std::vector<uchar> image_encode(Mat& src);
    void init_topics();

public:
    MqttHandle(Login_config& login_info, QObject *parent=nullptr);

    void connect_mqtt();
    QString topic(QString tp);
    void onMQTT_Received( QMQTT::Message message);
    void onMQTT_Error(QMQTT::ClientError error);
    void Publich_img(Mat& image);
    void QMQTT_connected();
    void Publish_data(QString topic, QString data);

signals:
    void mqttReceive(QMQTT::Message msg);
};

#endif // MQTTHANDLE_H
