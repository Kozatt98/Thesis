import pyqtgraph as pg
from PyQt5.QtCore import QTimer


class Display(pg.PlotWidget):
    def __init__(self):
        super(Display, self).__init__()
        self.data_in = None
        self.data = []
        self.data2 = []
        self.data3 = []

        self.setBackground("k")
        self.showGrid(x=True, y=True)

        self.plot_changed()

        self.pen1 = pg.mkPen(width=2, color=(255, 0, 0))
        self.pen2 = pg.mkPen(width=2, color=(0, 0, 255))
        self.pen3 = pg.mkPen(width=2, color=(0, 255, 0))

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
        self.setXRange(-900, 901)
        self.setYRange(255, 0)

    def clear_buffer(self):
        self.data.clear()
        self.data2.clear()
        self.data3.clear()

    def refresh_plots(self):
        if len(self.data) > 2048:
            self.plot1.setData(self.x_axis, self.data[-2047:], pen=self.pen1)
        if len(self.data2) > 2048:
            self.plot2.setData(self.x_axis, self.data2[-2047:], pen=self.pen2)
        if len(self.data3) > 2048:
            self.plot3.setData(self.x_axis, self.data3[-2047:], pen=self.pen3)

        if len(self.data) > 1100000:
            self.data = self.data[-1000000:]
        if len(self.data2) > 1100000:
            self.data = self.data2[-1000000:]
        if len(self.data3) > 1100000:
            self.data = self.data3[-1000000:]

    def start_refreshing(self):
        self.timer.start()

    def stop_refreshing(self):
        self.timer.stop()
