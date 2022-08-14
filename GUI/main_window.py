import csv
import struct
from datetime import datetime
from display import Display
from worker import Worker
from channel_settings import ChannelSettings

from serial_com import SerialCom
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *


class Window(QWidget):
    def __init__(self):
        super().__init__()

        self.serial_com = SerialCom()  # Todo(Attila): change to serial_com

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
        self.clear_btn = QPushButton(self)
        self.clear_btn.setText("Clear buffer")
        self.save_btn.setText("Save")
        self.save_btn.clicked.connect(self.save)
        self.clear_btn.clicked.connect(self.display.clear_buffer)
        self.saveName = QLineEdit(self)
        self.saveName.setText("measurement {}.csv".format(datetime.now().strftime("%d-%m-%Y")))

        self.ranges = ["±1V", "±2V", "±4V", "±5V", "±10V", "±20V"]
        self.select_range = QComboBox()
        self.select_range.addItems(self.ranges)
        self.select_range.currentIndexChanged.connect(self.range_change)

        self.channel_settings = ChannelSettings()

        vbox = QVBoxLayout()
        vbox.addLayout(self.channel_settings)
        vbox.addWidget(self.clear_btn)
        vbox.addWidget(self.display)
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
            data_len = len(self.display.data)

            for i in range(0, data_len):
                writer.writerow([i, self.display.data[i]])

        self.display.data.clear()
        self.display.data2.clear()
        self.display.data3.clear()

    def add_data(self):
        try:
            while self.serial_com.isOpen():
                incoming = self.serial_com.read(2048)
                incoming = list(incoming)
                if len(incoming) == 0:
                    continue

                # print(incoming)
                print(len(incoming))
                check_value = incoming[0]
                incoming = incoming[1:]
                if check_value == 0:
                    self.display.data.extend(incoming)
                if check_value == 1:
                    self.display.data2.extend(incoming)
                if check_value == 2:
                    self.display.data3.extend(incoming)

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
        self.serial_com.close()
        self.timer.stop()

    def range_change(self, i):
        print("Current index {}".format(i))
        print("Current selection: {}".format(self.ranges[i]))
        if self.serial_com.isOpen():
            struct_ = struct.pack('!B', self.select_range.currentIndex())
            print(struct_)
            self.serial_com.write(struct_)

    def search_serial_com(self):
        if self.serial_com.search_serial():
            self.timer.stop()
            self.display.start_refreshing()
            self.threadpool.start(self.worker)
            print("Serial found")

