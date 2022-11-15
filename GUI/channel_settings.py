from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
import numpy as np


class ChannelSettings(QHBoxLayout):

    def __init__(self, send_function, set_range_function):
        super(ChannelSettings, self).__init__()
        self.send_function = send_function
        self.set_range_function = set_range_function
        self.__voltage_ranges = ["±1V", "±2V", "±4V", "±5V", "±10V", "±20V", "OFF"]
        self.__current_ranges = ["20mA", "OFF"]
        self.font_textfield = QFont("Arial", 20, weight=QFont.Bold)
        self.font_ranges = QFont("Arial", 14, weight=QFont.Decorative)
        self.text_field_size_policy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        self.min_width = 150

        self.channel1_range = QComboBox()
        self.channel2_range = QComboBox()
        self.channel3_range = QComboBox()

        self.text_field1 = QLabel()
        self.text_field1.setAlignment(Qt.AlignCenter)
        self.text_field1.setText("A")
        self.text_field1.setFont(self.font_textfield)
        self.text_field1.setStyleSheet("color: #0000FF")
        self.text_field1.setMinimumWidth(self.min_width)
        self.channel1_range.addItems(self.__voltage_ranges)
        self.channel1_range.setFont(self.font_ranges)
        self.channel1_range.setMinimumWidth(self.min_width)

        self.text_field2 = QLabel()
        self.text_field2.setAlignment(Qt.AlignCenter)
        self.text_field2.setText("B")
        self.text_field2.setFont(self.font_textfield)
        self.text_field2.setStyleSheet("color: #FF0000")
        self.text_field2.setMinimumWidth(self.min_width)
        self.channel2_range.addItems(self.__voltage_ranges)
        self.channel2_range.setFont(self.font_ranges)
        self.channel2_range.setMinimumWidth(self.min_width)

        self.text_field3 = QLabel()
        self.text_field3.setAlignment(Qt.AlignCenter)
        self.text_field3.setText("C")
        self.text_field3.setFont(self.font_textfield)
        self.text_field3.setStyleSheet("color: #00FF00")
        self.text_field3.setMinimumWidth(self.min_width)
        self.channel3_range.addItems(self.__current_ranges)
        self.channel3_range.setFont(self.font_ranges)
        self.channel3_range.setMinimumWidth(self.min_width)

        self.horizontal_spacer = QSpacerItem(30, 20, hPolicy=QSizePolicy.Policy.Fixed,
                                             vPolicy=QSizePolicy.Policy.Minimum)

        self.horizontal_spacer_end = QSpacerItem(200, 20, hPolicy=QSizePolicy.Policy.Expanding,
                                                 vPolicy=QSizePolicy.Policy.Minimum)

        self.addItem(self.horizontal_spacer)
        self.addWidget(self.text_field1)
        self.addWidget(self.channel1_range)
        self.addItem(self.horizontal_spacer)
        self.addWidget(self.text_field2)
        self.addWidget(self.channel2_range)
        self.addItem(self.horizontal_spacer)
        self.addWidget(self.text_field3)
        self.addWidget(self.channel3_range)
        self.addItem(self.horizontal_spacer_end)
        self.addItem(self.horizontal_spacer_end)

        self.channel1_range.setCurrentIndex(len(self.__voltage_ranges) - 1)
        self.channel2_range.setCurrentIndex(len(self.__voltage_ranges) - 1)
        self.channel3_range.setCurrentIndex(len(self.__current_ranges) - 1)

        self.channel1_range.currentIndexChanged.connect(self.on_combobox_changed)
        self.channel2_range.currentIndexChanged.connect(self.on_combobox_changed)
        self.channel3_range.currentIndexChanged.connect(self.on_combobox_changed)


    def on_combobox_changed(self):
        data_to_send = np.zeros(3, dtype="B")
        data_to_send[0] = self.channel1_range.currentIndex()
        data_to_send[1] = self.channel2_range.currentIndex()
        data_to_send[2] = self.channel3_range.currentIndex()
        self.send_function(data_to_send)
        self.set_range_function(256, 1024, 2048)

        # TODO(Attila): Call func with appropriate bytearray according to selection, add connect to combo boxes
        print("Combo changed")
        print(data_to_send)
        print("")
