from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt


class ChannelSettings(QHBoxLayout):
    def __init__(self):
        super(ChannelSettings, self).__init__()
        self.__voltage_ranges = ["±1V", "±2V", "±4V", "±5V", "±10V", "±20V"]
        self.__current_ranges = []
        self.__voltage_channel_count = 2
        self.__current_channel_count = 1

        self.channel1_range = QComboBox()
        self.channel2_range = QComboBox()
        self.channel3_range = QComboBox()

        self.text_field1 = QLabel()   # TODO(Attila): more labels, and fancy up
        self.text_field1.setText("A")
        self.channel1_range.addItems(self.__voltage_ranges)

        self.text_field2 = QLabel()
        self.text_field2.setText("B")
        self.channel2_range.addItems(self.__voltage_ranges)

        self.text_field3 = QLabel()
        self.text_field3.setText("C")
        self.channel3_range.addItems(self.__current_ranges)

        self.addWidget(self.text_field1)
        self.addWidget(self.channel1_range)
        self.addWidget(self.text_field2)
        self.addWidget(self.channel2_range)
        self.addWidget(self.text_field3)
        self.addWidget(self.channel3_range)
