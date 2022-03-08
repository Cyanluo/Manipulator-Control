from networkHandle.NetHandle import MqttDevMonitor

import time
import cv2 as cv

class ServerMonitor:
    def __init__(self):
        self.pcClient = MqttDevMonitor()

    def pub(self):
        self.pcClient.publish_data("/pc/dataChannel1/1", "3;-20,-30,50,-10,0")
        self.pcClient.publish_data("/pc/command/1", "0")

    def img_process(self):
        pass


def main():
    server = ServerMonitor()

    while True:
        time.sleep(20)
        server.pub()

if __name__ == '__main__':
    main()
