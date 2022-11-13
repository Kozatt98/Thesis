from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont


class ChannelSettings(QHBoxLayout):
    def __init__(self, func):
        super(ChannelSettings, self).__init__()
        self.func = func
        self.__voltage_ranges = ["±1V", "±2V", "±4V", "±5V", "±10V", "±20V"]
        self.__current_ranges = []
        self.font = QFont("Arial", 20, weight=QFont.Bold)
        self.text_field_size_policy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        self.min_width = 150

        self.channel1_range = QComboBox()
        self.channel2_range = QComboBox()
        self.channel3_range = QComboBox()

        self.text_field1 = QLabel()
        self.text_field1.setAlignment(Qt.AlignCenter)
        self.text_field1.setText("A")
        self.text_field1.setFont(self.font)
        self.text_field1.setMinimumWidth(self.min_width)
        self.channel1_range.addItems(self.__voltage_ranges)
        self.channel1_range.setFont(self.font)
        self.channel1_range.setMinimumWidth(self.min_width)

        self.text_field2 = QLabel()
        self.text_field2.setAlignment(Qt.AlignCenter)
        self.text_field2.setText("B")
        self.text_field2.setFont(self.font)
        self.text_field2.setMinimumWidth(self.min_width)
        self.channel2_range.addItems(self.__voltage_ranges)
        self.channel2_range.setFont(self.font)
        self.channel2_range.setMinimumWidth(self.min_width)

        self.text_field3 = QLabel()
        self.text_field3.setAlignment(Qt.AlignCenter)
        self.text_field3.setText("C")
        self.text_field3.setFont(self.font)
        self.text_field3.setMinimumWidth(self.min_width)
        self.channel3_range.addItems(self.__current_ranges)
        self.channel3_range.setFont(self.font)
        self.channel3_range.setMinimumWidth(self.min_width)

        self.horizontal_spacer = QSpacerItem(30, 20, hPolicy=QSizePolicy.Policy.Fixed, vPolicy=QSizePolicy.Policy.Minimum)

        self.horizontal_spacer_end = QSpacerItem(200, 20, hPolicy=QSizePolicy.Policy.Expanding, vPolicy=QSizePolicy.Policy.Minimum)

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

    def channel_change(self):
        # TODO(Attila): Call func with appropriate bytearray according to selection, add connect to combo boxes
        # self.func()
        pass
