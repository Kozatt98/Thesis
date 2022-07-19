from PyQt5.QtWidgets import *


class ChannelSettings(QHBoxLayout):
    def __init__(self):
        super(ChannelSettings, self).__init__()
        self.ranges = ["±1V", "±2V", "±4V", "±5V", "±10V", "±20V"]

        self.channel1_range = QComboBox()
        self.channel2_range = QComboBox()
        self.channel3_range = QComboBox()

        self.channel1_range.addItems(self.ranges)
        self.channel2_range.addItems(self.ranges)
        self.channel3_range.addItems(self.ranges)

        self.text_field = QLabel()   # TODO(Attila): more labels, and fancy up
        self.text_field.setText("A")

        self.addWidget(self.text_field)
        self.addWidget(self.channel1_range)
        self.addWidget(self.channel2_range)
        self.addWidget(self.channel3_range)