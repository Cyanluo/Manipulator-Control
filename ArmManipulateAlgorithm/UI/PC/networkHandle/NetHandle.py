import sys
sys.path.append("..")
from paho.mqtt import client as mqtt_client
from handler.config_handler import CfgInfo
from handler.global_logger import logger
import time
import cv2
import numpy as np
import queue as Queue
from base64 import b64encode, b64decode
import matplotlib.pyplot as plt

class MqttDevMonitor():
    def __init__(self, mxa_img_queue_buff=20):
        self.m_client = dict()
        self.cfg = CfgInfo()
        self.managerClient = self.clientManageConnect()
        self.imageQueue = Queue.Queue(maxsize=mxa_img_queue_buff)

    def clientManageConnect(self):
        client = self.connect("server-manager-client")

        self.subscribe(client, "$SYS/brokers/+/clients/+/connected")
        self.subscribe(client, "$SYS/brokers/+/clients/+/disconnected")
        self.subscribe(client, self.cfg.getMqttSubTopic()["image"][:-10] + "+")

        # test
        self.subscribe(client, "/armControllerNode/dataChannel1/1234")
        self.subscribe(client, "/armControllerNode/busy/1234")

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

        elif msg.topic.startswith("/client/image/"):
            client_id = msg.topic.split('/')[-1]

            try:
                #print(msg.payload)
                # img_original = b64decode(msg.payload)
                # img_np = np.frombuffer(img_original, dtype=np.uint8)
                # img = cv2.imdecode(img_np, cv2.IMREAD_UNCHANGED)

                data = np.frombuffer(msg.payload, np.uint8)
                img = cv2.imdecode(data, 1)  # 解码处理
                self.imageQueue.put([client_id, img])
                #cv2.imshow("", img)
                #cv2.waitKey(0)
            except Exception as e:
                logger.error(e)
                logger.error("Fail to convert data to img !!!")

        elif msg.topic.startswith("/armControllerNode/dataChannel1/"):
            print(msg.payload.decode())
            data = msg.payload.decode()
            l_data = data[:-1].split(';')
            x = list()
            y = list()
            for each in l_data:
                x.append(float((each.split(','))[0]))
                y.append(float((each.split(','))[1]))
            fig = plt.figure()
            a1 = fig.add_subplot(111)
            a1.set_ylim(0, 100)
            a1.set_xlim(0, 100)
            a1.plot(x, y)
            a1.grid(True)
            plt.show()

        elif msg.topic.startswith("/armControllerNode/busy/"):
            print(msg.payload.decode())


    def subscribe(self, client: mqtt_client, topic):
        client.subscribe(topic)

    def publish(self, client, topic, data):
        result = client.publish(topic, data)
        # result: [0, 1]
        status = result[0]

    def publish_data(self, topic, data):
        self.publish(self.managerClient, topic, data)

    def get_image(self):
        return self.imageQueue.get()

    def has_img(self):
        return not self.imageQueue.empty()


def main():
    testclient = MqttDevMonitor()

    while True:
        time.sleep(0.02)


if __name__ == '__main__':
    main()
