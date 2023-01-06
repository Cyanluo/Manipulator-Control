import matplotlib.pyplot  as plt
from PyQt5.QtCore import QObject

class MyPlot(QObject):
    def __init__(self, parent=None):
        super(MyPlot, self).__init__(parent)
        self.initPlot()
        self.lineCount = 1
        self.nLineFlag = True
        self.colorList = ['green', 'green', 'blue', 'black', 'yellow']
        self.labelList = None

    def clear(self):
        plt.close(self.figure)
        self.initPlot()

    def initPlot(self):
        self.figure = plt.figure()
        self.a1 = self.figure.add_subplot(121)
        self.a2 = self.figure.add_subplot(122)
        self.a1.set_xlabel("times")
        self.a1.set_ylabel("fitness")
        self.a2.set_xlabel("times")
        self.a2.set_ylabel("inaccuracy")
        self.a1.set_ylim(0, 100)
        self.a1.set_xlim(0, 100)
        self.a1.grid(True)
        self.a2.set_ylim(0, 1)
        self.a2.set_xlim(0, 100)
        self.a2.grid(True)

    def decodeStr(self, data):
        l_data = data[:-1].split(';')
        x = list()
        fitness = list()
        inac = list()
        for each in l_data:
            x.append(float((each.split(','))[0]))
            fitness.append(float((each.split(','))[1]))
            inac.append(float((each.split(','))[2]))

        return [x, fitness, inac]

    def loadOneLine(self, x, fitness, inaccuracy, label, color):
        self.a1.plot(x, fitness, color=color, linewidth=1, label=label)
        self.a2.plot(x, inaccuracy, color=color, linewidth=1, label=label)

    def drawOneLine(self, x, fitness, inaccuracy, label):
        self.clear()
        self.loadOneLine(x, fitness, inaccuracy, label, 'blue')
        self.plotDisp()

    def drawNLine(self, n, x, fitness, inaccuracy, label):
        if self.nLineFlag:
            self.nLineFlag = False
            self.lineCount = n
            self.labelList = label
            self.labelList.reverse()
            self.clear()

        self.loadOneLine(x, fitness, inaccuracy, self.labelList[self.lineCount-1], self.colorList[self.lineCount])
        self.lineCount -= 1

        if self.lineCount < 1:
            self.lineCount = 1
            self.nLineFlag = True
            self.labelList = None
            self.plotDisp()

    def draw(self, param):
        if param.__len__() == 4:
            self.drawOneLine(param[0], param[1], param[2], param[3])
        elif param.__len__() == 5:
            self.drawNLine(param[0], param[1], param[2], param[3], param[4])
        else:
            pass

    def plotDisp(self):
        self.a1.legend()
        self.a2.legend()
        plt.show()

if __name__ == '__main__':
    demo = MyPlot()
