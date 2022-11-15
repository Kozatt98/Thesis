import pyqtgraph as pg
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
        self.a_y_axis = []
        self.b_y_axis = []
        self.c_y_axis = []

        for i in range(-1024, 1023):
            self.x_axis.append(i)

        self.plot1 = self.plot(self.x_axis, self.a_y_axis, pen=self.pen1)
        self.plot2 = self.plot(self.x_axis, self.b_y_axis, pen=self.pen2)
        self.plot3 = self.plot(self.x_axis, self.c_y_axis, pen=self.pen3)

    def plot_changed(self):
        # self.setXRange(0, 2047)
        # self.setYRange(255, 0)
        pass

    def refresh_plots(self):
        self.plot1.setData(self.data, pen=self.pen1)
        self.plot2.setData(self.data2, pen=self.pen2)
        self.plot3.setData(self.data3, pen=self.pen3)

    def set_ranges(self, a_range, b_range, c_range):
        pass
