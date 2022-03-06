from networkHandle.NetHandle import MqttDevMonitor

import time
import cv2 as cv

class ServerMonitor:
    def __init__(self):
        self.pcClient = MqttDevMonitor()

    def pub(self):
        self.pcClient.publish_data("/pc/command/1", b'\x01')

    def img_process(self):
        pass


def main():
    server = ServerMonitor()

    while True:
        time.sleep(10)
        server.pub()

if __name__ == '__main__':
    main()
