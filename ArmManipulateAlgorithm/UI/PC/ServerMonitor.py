from networkHandle.NetHandle import MqttDevMonitor

import time
import cv2 as cv

class ServerMonitor:
    def __init__(self):
        self.pcClient = MqttDevMonitor()
        cv.namedWindow('LRImage')
        cv.setMouseCallback('LRImage', self.selectObject)
        self.cleft = None
        self.cright = None
        self.triF = False
        self.reset = False
        self.displayF = True
        self.count = 0
        self.img = None

    def pub(self):
        self.pcClient.publish_data("/pc/dataChannel1/1", "7;0,0,0,0,0,0")
        self.pcClient.publish_data("/pc/command/1", "0")
        # self.pcClient.publish_data("/pc/dataChannel1/1", "8;0,1,0,294,1,0,0,-81,0,0,-1,10,0,0,0,1")
        # self.pcClient.publish_data("/pc/command/1", "1")

    def reachDist(self):
        pointData = str(self.cleft[0]) + "," + str(self.cleft[1]) + "," + str(self.cright[0]) + "," + str(self.cright[1])
        self.pcClient.publish_data("/pc/dataChannel1/1", "9;" + pointData)
        print(pointData)
        flag, xyz = self.pcClient.getXYZ()
        repeat = 0
        while not flag:
            flag, xyz = self.pcClient.getXYZ()
            repeat += 1
            time.sleep(1)
            if repeat > 2:
                print("send error!!!")
                return

        target = "8;0,1,0," + str(-xyz[2]+40) + ",1,0,0, " + str(xyz[1]+20) + " ,0,0,-1," + str(-xyz[0]+240) + ",0,0,0,1"
        print(target)
        self.pcClient.publish_data("/pc/dataChannel1/1", target)
        self.pcClient.publish_data("/pc/command/1", "1")

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
                    self.reachDist()

                if self.reset:
                    self.reset = False
                    self.pcClient.publish_data("/pc/dataChannel1/1", "7;0,0,0,0,0,0")
                    self.pcClient.publish_data("/pc/command/1", "0")
            else:
                cv.waitKey(200)

def main():
    server = ServerMonitor()
    server.img_process()

    while True:
        cv.waitKey(500)

if __name__ == '__main__':
    main()
