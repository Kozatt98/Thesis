import pyqtgraph as pg
from PyQt5.QtCore import QTimer
import numpy as np


class Display(pg.PlotWidget):
    def __init__(self):
        super(Display, self).__init__()
        self.data_in = None
        self.data = np.zeros(shape=1023, dtype="u1")
        self.data2 = np.zeros(shape=1023, dtype="u1")
        self.data3 = np.zeros(shape=1023, dtype="u1")


        self.setBackground("w")
        self.showGrid(x=True, y=True)

        self.plot_changed()

        self.pen1 = pg.mkPen(width=2, color=(0, 255, 0))
        self.pen2 = pg.mkPen(width=2, color=(255, 0, 0))
        self.pen3 = pg.mkPen(width=2, color=(0, 0, 255))

        self.x_axis = []
        self.y_axis = []

        self.plot1 = self.plot(self.x_axis, self.y_axis, pen=self.pen1)
        self.plot2 = self.plot(self.x_axis, self.y_axis, pen=self.pen2)
        self.plot3 = self.plot(self.x_axis, self.y_axis, pen=self.pen3)

        self.sigRangeChanged.connect(self.plot_changed)

        for i in range(-1024, 1023):
            self.x_axis.append(i)

        self.timer = QTimer()
        self.timer.setInterval(15)
        self.timer.timeout.connect(self.refresh_plots)

    def plot_changed(self):
        # self.setXRange(0, 2047)
        # self.setYRange(255, 0)
        pass


    def refresh_plots(self):
        self.plot1.setData(self.data, pen=self.pen1)
        self.plot2.setData(self.data2, pen=self.pen2)
        self.plot3.setData(self.data3, pen=self.pen3)


    def start_refreshing(self):
        self.timer.start()

    def stop_refreshing(self):
        self.timer.stop()
