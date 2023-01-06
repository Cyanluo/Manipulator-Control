import sys
import matplotlib.pyplot  as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import  QApplication,  QVBoxLayout, QWidget

class MyPlot(QWidget):
    def __init__(self):
        super(MyPlot, self).__init__(parent=None)

        layout = QVBoxLayout(self)

        self.figure = plt.figure()
        self.canves = FigureCanvas(self.figure)
        self.layout().addWidget(self.canves)
        self.initPlot()

    def clear(self):
        self.figure.clf()
        self.initPlot()
        self.plotShow()

    def plotShow(self):
        plt.show()
        self.canves.draw()

    def initPlot(self):
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
        self.canves.draw()

    def draw(self, x, fitness, inaccuracy,label):
        if label == "PSO":
            self.a1.plot(x, fitness, color='green', linewidth=1, label=label)
            self.a2.plot(x, inaccuracy, color='green', linewidth=1, label=label)
        elif label == "GA_PSO":
            self.a1.plot(x, fitness, color='blue', linewidth=1, label=label)
            self.a2.plot(x, inaccuracy, color='blue', linewidth=1, label=label)

        self.a1.legend()
        self.a2.legend()

        self.plotShow()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    demo = MyPlot()
    demo.show()

    sys.exit(app.exec_())
