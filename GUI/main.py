import csv
import struct
import sys
import traceback
from datetime import datetime

import pyqtgraph as pg
import serial
import serial.tools.list_ports as serial_ports
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *


def main():
    app = QApplication(sys.argv)
    window = Window()
    app.aboutToQuit.connect(window.cleanup)
    window.showMaximized()
    sys.exit(app.exec())


class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.ser = serial.Serial()
        self.is_serial_open = False
        self.__setup_serial()

        self.data_in = None
        self.data = []
        self.data2 = []
        self.data3 = []
        self.setGeometry(200, 200, 800, 600)
        self.setWindowTitle("Scope")

        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground("w")
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setXRange(1024, -1024)
        self.graphWidget.setYRange(255, 0)

        self.pen = pg.mkPen(color=(255, 0, 0))
        self.pen2 = pg.mkPen(color=(0, 0, 255))
        self.pen3 = pg.mkPen(color=(0, 255, 0))

        self.graphWidget.sigRangeChanged.connect(self.plot_changed)

        self.x_axis = []
        self.y_axis = []
        for i in range(-1024, 1023):
            self.x_axis.append(i)

        self.plot1 = self.graphWidget.plot(self.x_axis, self.y_axis, pen=self.pen)
        self.plot2 = self.graphWidget.plot(self.x_axis, self.y_axis, pen=self.pen2)
        self.plot3 = self.graphWidget.plot(self.x_axis, self.y_axis, pen=self.pen3)

        self.timer = QTimer()
        self.timer.setInterval(15)
        self.timer.timeout.connect(self.__search_serial)
        self.timer.start()

        self.threadpool = QThreadPool(parent=self)

        self.worker2 = Worker(self.add_data)
        self.worker2.signals.error.connect(self.error_handler)

        self.save_btn = QPushButton(self)
        self.clear_btn = QPushButton(self)
        self.clear_btn.setText("Clear buffer")
        self.save_btn.setText("Save")
        self.save_btn.clicked.connect(self.save)
        self.clear_btn.clicked.connect(self.clear_buffer)
        self.saveName = QLineEdit(self)
        self.saveName.setText("measurement {}.csv".format(datetime.now().strftime("%d-%m-%Y")))

        self.ranges = ["±1V", "±2V", "±4V", "±5V", "±10V", "±20V"]
        self.select_range = QComboBox()
        self.select_range.addItems(self.ranges)
        self.select_range.currentIndexChanged.connect(self.range_change)

        vbox = QVBoxLayout()
        vbox.addWidget(self.clear_btn)
        vbox.addWidget(self.graphWidget)
        vbox.addWidget(self.saveName)
        vbox.addWidget(self.save_btn)
        vbox.addWidget(self.select_range)

        self.setLayout(vbox)

    def save(self):
        name = "measurement {}.csv".format(datetime.now().strftime("%d-%m-%Y"))
        if self.saveName.text() != "":
            name = self.saveName.text()
            if not name.endswith(".csv"):
                name = name + ".csv"

        with open(name, "w", newline='') as file:
            csv.register_dialect('magyar', delimiter=';')
            dialect = csv.get_dialect("magyar")
            writer = csv.writer(file, dialect=dialect)
            data_len = len(self.data)

            for i in range(0, data_len):
                writer.writerow([i, self.data[i]])

        self.data.clear()
        pass

    def clear_buffer(self):
        self.data.clear()
        self.data2.clear()
        self.data3.clear()

    def plot_changed(self):
        self.graphWidget.setXRange(1024, -1024)
        self.graphWidget.setYRange(255, 0)

    def add_data(self):
        try:
            while self.ser.isOpen():
                incoming = self.ser.read(2048)
                incoming = list(incoming)
                if len(incoming) == 0:
                    continue

                check_value = incoming[0]
                incoming = incoming[1:]
                if check_value == 0:
                    self.data.extend(incoming)
                if check_value == 1:
                    self.data2.extend(incoming)
                if check_value == 2:
                    self.data3.extend(incoming)
        except:
            self.ser.close()

    def refresh_graph(self):
        if len(self.data) > 2048:
            self.plot1.setData(self.x_axis, self.data[-2047:], pen=self.pen)
        if len(self.data2) > 2048:
            self.plot2.setData(self.x_axis, self.data2[-2047:], pen=self.pen2)
        if len(self.data3) > 2048:
            self.plot3.setData(self.x_axis, self.data3[-2047:], pen=self.pen3)

    def __setup_serial(self):
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 1

    def __search_serial(self):
        if not self.is_serial_open:
            ports = serial_ports.comports()
            for p in ports:
                print(p.name)
                if (p.vid == 1155 and p.pid == 22336) or p.name == "COM2":
                    self.ser.port = p.name
                    self.ser.open()
                    if self.ser.isOpen():
                        print("Connected to device on port ", p.name)
                        self.is_serial_open = True
                        self.timer.timeout.connect(self.refresh_graph)
                        self.serial_found()

    def serial_found(self):
        print("Serial found")
        print(self.select_range.currentIndex())
        bytes_ = bytes(self.select_range.currentIndex())
        struct_ = struct.pack('!B', self.select_range.currentIndex())
        print(struct_)
        self.ser.write(struct_)
        self.timer.timeout.connect(self.refresh_graph)
        self.threadpool.start(self.worker2)
        if len(self.data) > 2048:
            self.timer.start()

    def error_handler(self, error):
        self.ser.close()
        self.threadpool.cancel(self.worker2)
        print(error)

    def cleanup(self):
        print("cleanup")
        self.ser.close()
        self.timer.stop()

    def range_change(self, i):
        print("Current index {}".format(i))
        print("Current selection: {}".format(self.ranges[i]))
        if self.ser.isOpen():
            struct_ = struct.pack('!B', self.select_range.currentIndex())
            print(struct_)
            self.ser.write(struct_)


class Worker(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()

    @pyqtSlot()
    def run(self):
        try:
            self.fn(*self.args, **self.kwargs)

        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))


class WorkerSignals(QObject):
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    result = pyqtSignal(object)


if __name__ == '__main__':
    main()
