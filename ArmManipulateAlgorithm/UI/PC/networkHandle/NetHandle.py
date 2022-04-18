import sys
sys.path.append("..")
from paho.mqtt import client as mqtt_client
from handler.config_handler import CfgInfo
from handler.global_logger import logger
import time
import cv2
import numpy as np
import queue as Queue
from MyPlot import MyPlot
from PyQt5.QtCore import QThread, QMutex, pyqtSignal, QObject
from CONST import MODE

class MqttDevMonitor(QObject):
    plotData = pyqtSignal(list)
    onPublish = pyqtSignal(list)
    xyzReady = pyqtSignal(list)

    def __init__(self, mxa_img_queue_buff=5, parent=None):
        super(MqttDevMonitor, self).__init__(parent)
        mqtt_client.Client._call_socket_register_write = lambda _self: None
        mqtt_client.Client._call_socket_unregister_write = lambda _self, _sock=None: None
        self.m_client = dict()
        self.cfg = CfgInfo()
        self.managerClient = self.clientManageConnect()
        self.imageQueue = Queue.Queue(maxsize=mxa_img_queue_buff)
        self.deepImageQueue = Queue.Queue(maxsize=mxa_img_queue_buff)

        self.myPlot = MyPlot()
        self.mode = MODE.NormalRun

        self.plotThread = QThread()
        self.myPlot.moveToThread(self.plotThread)
        self.plotData.connect(self.myPlot.draw)
        self.plotThread.start()

        self.publishThread = QThread()
        self.moveToThread(self.publishThread)
        self.onPublish.connect(self.publishEmitSignal)
        self.publishThread.start()

    def clientManageConnect(self):
        client = self.connect("server-manager-client")

        self.subscribe(client, "$SYS/brokers/+/clients/+/connected")
        self.subscribe(client, "$SYS/brokers/+/clients/+/disconnected")
        self.subscribe(client, self.cfg.getMqttSubTopic()["image"][:-10] + "+")
        self.subscribe(client, "/armControllerNode/dataChannel1/1234")
        self.subscribe(client, "/armControllerNode/busy/1234")
        self.subscribe(client, "/armControllerNode/LRImage/1234")
        self.subscribe(client, "/armControllerNode/deepMap/1234")

        client.on_message = self.on_manager_message
        client.loop_start()

        return client

    def connect(self, client_id):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                logger.info("%s: Connected to MQTT Broker!" % client_id)
            else:
                logger.error("%s: Failed to connect, return code %d\n" % (client_id, rc))

        def on_disconnect(client, userdata, flags, rc):
            if rc == 0:
                logger.info("%s: Disconnected to MQTT Broker!" % client_id)
            else:
                logger.error("%s: Failed to Disconnect, return code %d\n" % (client_id, rc))

        address = self.cfg.getMqttAddress()

        client = mqtt_client.Client(client_id)
        client.on_connect = on_connect
        client.connect(address["host"], int(address["port"]))

        return client

    def control_client_connect(self, client_id):
        client = self.connect(client_id)
        self.m_client[client_id] = client
        client.loop_start()

        return client

    def on_manager_message(self, client, userdata, msg):
        if msg.topic.endswith("connected"):
            logger.info(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

        elif msg.topic.endswith("disconnected"):
            logger.info(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

        elif msg.topic.startswith("/armControllerNode/LRImage/"):
            client_id = msg.topic.split('/')[-1]

            try:
                data = np.frombuffer(msg.payload, np.uint8)
                img = cv2.imdecode(data, 1)  # 解码处理
                self.imageQueue.put([client_id, img])
            except Exception as e:
                logger.error(e)
                logger.error("Fail to convert data to img !!!")

        elif msg.topic.startswith("/armControllerNode/deepMap/"):
            client_id = msg.topic.split('/')[-1]

            try:
                data = np.frombuffer(msg.payload, np.uint8)
                img = cv2.imdecode(data, 1)  # 解码处理
                self.deepImageQueue.put([client_id, img])
            except Exception as e:
                logger.error(e)
                logger.error("Fail to convert data to img !!!")

        elif msg.topic.startswith("/armControllerNode/dataChannel1/"):
            print(msg.payload.decode())
            q_data = msg.payload.decode()
            (head, data) = q_data.split(":")

            if head.startswith("plotData"):
                if self.mode == MODE.TestAlgorithm:
                    param = self.myPlot.decodeStr(data)
                    param = [3,] + param
                    param.append(["GA_PSO", "GA", "PSO"])
                else:
                    param = self.myPlot.decodeStr(data)
                    param += ['GA_PSO']

                self.plotData.emit(param)

            elif head.startswith("pointXYZ"):
                points = data[:-1].split(',')
                xyz = list()
                for each in points:
                    xyz.append(float(each))

                self.xyzReady.emit(xyz)
                print("XYZ", xyz)

        elif msg.topic.startswith("/armControllerNode/busy/"):
            print(msg.payload.decode())

    def subscribe(self, client: mqtt_client, topic):
        client.subscribe(topic)

    def publish(self, client, topic, data):
        #result = client.publish(topic, data, qos=0)
        # result: [0, 1]
        self.onPublish.emit([client, topic, data])

    def publishEmitSignal(self, param):
        param[0].publish(param[1], param[2], qos=0)

    def publish_data(self, topic, data):
        self.publish(self.managerClient, topic, data)

    def get_image(self, imgType):
        if imgType == "LRImage":
            return self.imageQueue.get()
        elif imgType == "deepMap":
            return self.deepImageQueue.get()
        else:
            pass

    def has_img(self, imgType):
        if imgType == "LRImage":
            return not self.imageQueue.empty()
        elif imgType == "deepMap":
            return not self.deepImageQueue.empty()
        else:
            return False

    def setMode(self, mode):
        self.mode = mode


def main():
    testclient = MqttDevMonitor()

    while True:
        time.sleep(0.02)


if __name__ == '__main__':
    main()
