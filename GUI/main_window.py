import csv
from datetime import datetime

from display import Display
from worker import Worker
from channel_settings import ChannelSettings
import numpy as np

from serial_com import SerialCom
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *


class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_com = SerialCom()  # Todo(Attila): change to serial_com
        self.data_type = np.dtype("u1")
        self.setGeometry(200, 200, 800, 600)
        self.setWindowTitle("Scope")

        self.display = Display()

        self.timer = QTimer()
        self.timer.setInterval(15)
        self.timer.timeout.connect(self.search_serial_com)
        self.timer.start()

        self.threadpool = QThreadPool(parent=self)
        self.worker = Worker(self.add_data)
        self.worker.signals.error.connect(self.error_handler)

        self.save_btn = QPushButton(self)
        self.save_btn.setText("Save")
        self.save_btn.clicked.connect(self.save)
        self.saveName = QLineEdit(self)
        self.saveName.setText("measurement {}.csv".format(datetime.now().strftime("%d-%m-%Y")))

        self.trigger_slider = QSlider(Qt.Orientation.Vertical)
        self.trigger_slider.setRange(-2048, 2047)

        self.channel_settings = ChannelSettings(self.serial_com.send_settings, self.display.set_on, self.trigger_slider)


        self.hbox = QHBoxLayout()
        self.hbox.addWidget(self.display)
        self.hbox.addWidget(self.trigger_slider)

        vbox = QVBoxLayout()
        vbox.addLayout(self.channel_settings)
        vbox.addLayout(self.hbox)
        vbox.addWidget(self.saveName)
        vbox.addWidget(self.save_btn)
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
            data_len = len(self.display.data)

            for i in range(0, data_len):
                writer.writerow([i, self.display.data[i]])

        self.display.data.clear()
        self.display.data2.clear()
        self.display.data3.clear()

    def add_data(self):
        try:
            while self.serial_com.isOpen():
                if self.serial_com.inWaiting():
                    incoming = self.serial_com.read(500)
                    # print(len(incoming))
                    np_read = np.frombuffer(incoming, dtype="B", offset=0, count=500).view(dtype=self.data_type)
                    if len(incoming) != 0:
                        # print(np_read)
                        # print(len(np_read))
                        if np_read[0] == 0:
                            # print("A")
                            self.display.data = np_read[1:]
                        if np_read[0] == 1:
                            # print("B")
                            self.display.data2 = np_read[1:]
                        if np_read[0] == 2:
                            # print("C")
                            self.display.data3 = np_read[1:]
                        self.display.refresh_plots()

            self.serial_com.close()
            self.timer = QTimer()
            self.timer.setInterval(15)
            self.timer.timeout.connect(self.search_serial_com)
            self.timer.start()
            self.threadpool.cancel(self.worker)
        except:
            self.serial_com.close()

    def error_handler(self, error):
        self.serial_com.close()
        self.display.stop_refreshing()
        self.threadpool.cancel(self.worker)
        print(error)

    def cleanup(self):
        print("cleanup")
        # self.serial_com.reset_input_buffer()
        # self.serial_com.reset_output_buffer()
        # self.threadpool.cancel(self.worker)
        self.threadpool.clear()
        self.serial_com.close()
        self.timer.stop()
        self.destroy()

    # def range_change(self, i):
    #     print("Current index {}".format(i))
    #     print("Current selection: {}".format(self.ranges[i]))
    #     if self.serial_com.isOpen():
    #         struct_ = struct.pack('!B', self.select_range.currentIndex())
    #         print(struct_)
    #         self.serial_com.write(struct_)

    def search_serial_com(self):
        if self.serial_com.search_serial():
            self.timer.stop()
            self.threadpool.start(self.worker)
            print("Serial found")
