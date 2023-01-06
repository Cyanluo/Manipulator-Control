from networkHandle.NetHandle import MqttDevMonitor
import cv2 as cv
from CONST import ARM_STATUS, MODE
from PyQt5.QtCore import QCoreApplication, QObject
from PyQt5.QtWidgets import QApplication, QWidget
import sys
import json

import gi
from gi import require_version
require_version('Gtk', '2.0')
from gi.repository import Gtk

class ServerMonitor(QObject):
    def __init__(self):
        super(ServerMonitor, self).__init__(parent=None)
        self.pcClient = MqttDevMonitor()
        cv.namedWindow('LRImage')
        cv.setMouseCallback('LRImage', self.selectObject)
        self.pcClient.xyzReady.connect(self.reachDist)
        self.cleft = None
        self.cright = None
        self.triF = False
        self.reset = False
        self.displayF = True
        self.count = 0
        self.img = None

    def exeCMD(self, cmdID):
        self.pcClient.publish_data("/pc/command/1", str(int(cmdID)))

    def sendData(self, data):
        self.pcClient.publish_data("/pc/dataChannel1/1", json.dumps(data))

    def setting(self, setType, data):
        settingData = {
            "id": ARM_STATUS.SettingData,
            "setType": setType,
            "setData": data
        }
        self.sendData(settingData)

    def reachDist(self, xyz):
        target = {
            "id": ARM_STATUS.ReceiveTarget,
            "target":[[0, 0.98, 0.19, -xyz[2]+55],
                      [1,    0,    1, xyz[1]+5],
                      [0, 0.19,-0.98, -xyz[0]+240],
                      [0,    0,    0,    1]]
        }
        print("XYZ:", (-xyz[2] + 45, xyz[1], -xyz[0] + 225))
        print("-----------")
        self.sendData(target)
        self.exeCMD(ARM_STATUS.ReachTarget)

    def sendPoints(self):
        pointData = {
            "id": ARM_STATUS.ReceivePoints,
            "points": [list(self.cleft), list(self.cright)]
        }
        #pointData = str(self.cleft[0]) + "," + str(self.cleft[1]) + "," + str(self.cright[0]) + "," + str(self.cright[1])
        self.sendData(pointData)

    def selectObject(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.displayF = False
            if self.count == 0:
                self.cleft = (x, y)
                cv.putText(self.img, "p("+str(x)+","+str(y)+")", (x, y), cv.FONT_HERSHEY_SIMPLEX, 1, (200, 20, 150), 1, cv.LINE_AA)
                cv.imshow("LRImage", self.img)
            elif self.count == 1:
                self.cright = (x - 640, y)
                cv.putText(self.img, "p'("+str(x-640)+","+str(y)+")", (x, y), cv.FONT_HERSHEY_SIMPLEX, 1, (200, 20, 150), 1, cv.LINE_AA)
                cv.imshow("LRImage", self.img)

            self.count += 1
            if self.count > 2:
                self.count = 0
                self.triF = True
                self.displayF = True
            else:
                self.triF = False
        elif event == cv.EVENT_RBUTTONDOWN:
            self.reset = True

    def img_process(self):
        while True:
            if self.displayF:
                if self.pcClient.has_img("LRImage"):
                    self.img = self.pcClient.get_image("LRImage")[1]
                    cv.imshow("LRImage", self.img)
                    cv.waitKey(5)

                if self.pcClient.has_img("deepMap"):
                    cv.imshow("deepMap", self.pcClient.get_image("deepMap")[1])
                    cv.waitKey(5)

                if self.triF:
                    self.triF = False
                    self.setting("algorithmType", 0)
                    self.sendPoints()

                if self.reset:
                    self.reset = False
                    runParams = {
                        "id": ARM_STATUS.ReceiveRunParams,
                        "runParams": [0, 0, 0, 0, 0, 0]
                    }
                    self.sendData(runParams)
                    self.exeCMD(ARM_STATUS.ActionExecute)
            else:
                cv.waitKey(500)

def main():
    app = QApplication(sys.argv)
    server = ServerMonitor()
    server.setting("mode", MODE.NormalRun)
    # 0:ga_pso 1:ga 2:pso
    server.img_process()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
