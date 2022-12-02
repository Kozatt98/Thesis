import pyqtgraph as pg
import numpy as np


class Display(pg.PlotWidget):
    def __init__(self):
        super(Display, self).__init__()
        self.data_in = None
        self.data_type = np.dtype("u1")
        self.data = np.zeros(shape=500, dtype=self.data_type)
        self.data2 = np.zeros(shape=500, dtype=self.data_type)
        self.data3 = np.zeros(shape=500, dtype=self.data_type)

        self.is_on = [True, False, False]

        self.setBackground("w")
        self.showGrid(x=True, y=True)
        self.sigRangeChanged.connect(self.plot_changed)

        self.pen1 = pg.mkPen(width=2, color=(0, 0, 255))
        self.pen2 = pg.mkPen(width=2, color=(255, 0, 0))
        self.pen3 = pg.mkPen(width=2, color=(0, 255, 0))

        self.showAxes('left', False)
        self.showAxes('bottom', False)
        self.hideButtons()
        self.setLimits(xMin=-1, xMax=501, yMin=-0.51, yMax=0.51)

        self.plot1 = self.plot([], pen=self.pen1)
        self.plot2 = self.plot([], pen=self.pen2)
        self.plot3 = self.plot([], pen=self.pen3)

        self.setXRange(0, 501)
        self.setYRange(-0.5, 0.5)

    def plot_changed(self):
        self.setXRange(0, 501)
        self.setYRange(-0.5, 0.5)

    def refresh_plots(self):
        display_data1 = self.data / 2 ** 8 - 0.5
        display_data2 = self.data2 / 2 ** 8 - 0.5
        display_data3 = self.data3 / 2 ** 8 - 0.5

        if self.is_on[0]:
            self.plot1.setData(display_data1, pen=self.pen1)
        else:
            self.plot1.setData([-10])

        if self.is_on[1]:
            self.plot2.setData(display_data2, pen=self.pen2)
        else:
            self.plot2.setData([-10])

        if self.is_on[2]:
            self.plot3.setData(display_data3, pen=self.pen3)
        else:
            self.plot3.setData([-10])

    def set_on(self, a_range, b_range, c_range):
        self.is_on[0] = a_range
        self.is_on[1] = b_range
        self.is_on[2] = c_range
        self.refresh_plots()
