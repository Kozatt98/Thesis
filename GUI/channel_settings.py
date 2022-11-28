from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
import numpy as np


class ChannelSettings(QHBoxLayout):

    def __init__(self, send_function, set_range_function, trigger_slider):
        super(ChannelSettings, self).__init__()
        self.send_function = send_function
        self.set_is_on_function = set_range_function
        self.trigger_slider = trigger_slider

        self.trigger_slider.sliderReleased.connect(self.on_combobox_changed)

        self.__voltage_ranges = ["±1V", "±2V", "±4V", "±5V", "±10V", "±20V", "OFF"]
        self.__current_ranges = ["20mA", "OFF"]
        self.__trigger_options = ["Rising Edge", "Falling Edge", "None"]
        self.__timescale_options = ["1", "2", "3", "4", "5", "6", "7"]
        self.__trigger_channel_options = ["", "A", "B"]

        self.font_textfield = QFont("Arial", 20, weight=QFont.Bold)
        self.font_ranges = QFont("Arial", 14, weight=QFont.Decorative)
        self.text_field_size_policy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        self.min_width = 150

        self.channel1_range = QComboBox()
        self.channel2_range = QComboBox()
        self.channel3_range = QComboBox()
        self.trigger_select = QComboBox()
        self.trigger_channel = QComboBox()
        self.time_scale = QComboBox()

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

        self.text_field4 = QLabel()
        self.text_field4.setAlignment(Qt.AlignCenter)
        self.text_field4.setText("Trigger Options")
        self.text_field4.setFont(self.font_ranges)
        self.trigger_select.addItems(self.__trigger_options)
        self.trigger_select.setFont(self.font_ranges)
        self.trigger_select.setMinimumWidth(self.min_width)

        self.trigger_channel.addItems(self.__trigger_channel_options)
        self.trigger_channel.setFont(self.font_ranges)
        self.trigger_channel.setMinimumWidth(self.min_width)

        self.time_scale.addItems(self.__timescale_options)
        self.time_scale.setFont(self.font_ranges)
        self.time_scale.setMinimumWidth(self.min_width)

        self.horizontal_spacer = QSpacerItem(30, 20, hPolicy=QSizePolicy.Policy.Fixed,
                                             vPolicy=QSizePolicy.Policy.Minimum)

        self.horizontal_spacer = QSpacerItem(200, 20, hPolicy=QSizePolicy.Policy.Expanding,
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
        self.addItem(self.horizontal_spacer)
        self.addWidget(self.time_scale)
        self.addWidget(self.text_field4)
        self.addWidget(self.trigger_select)
        self.addWidget(self.trigger_channel)
        self.addItem(self.horizontal_spacer)

        self.channel1_range.setCurrentIndex(len(self.__voltage_ranges) - 2)
        self.channel2_range.setCurrentIndex(len(self.__voltage_ranges) - 1)
        self.channel3_range.setCurrentIndex(len(self.__current_ranges) - 1)
        self.trigger_select.setCurrentIndex(len(self.__trigger_options) - 1)

        self.channel1_range.currentIndexChanged.connect(self.on_combobox_changed)
        self.channel2_range.currentIndexChanged.connect(self.on_combobox_changed)
        self.channel3_range.currentIndexChanged.connect(self.on_combobox_changed)
        self.trigger_select.currentIndexChanged.connect(self.on_combobox_changed)
        self.trigger_channel.currentIndexChanged.connect(self.on_combobox_changed)
        self.time_scale.currentIndexChanged.connect(self.on_combobox_changed)

    def on_combobox_changed(self):
        trigger_value_to_send = self.trigger_slider.value() + 2048
        tigger_setting_h, tigger_setting_l = trigger_value_to_send >> 8, trigger_value_to_send & 0xFF
        data_to_send = np.ones(9, dtype="B")

        data_to_send[0] = self.channel1_range.currentIndex()    # Channel 1
        data_to_send[1] = self.channel2_range.currentIndex()    # Channel 2
        data_to_send[2] = self.channel3_range.currentIndex()    # Channel 3
        data_to_send[3] = self.trigger_select.currentIndex()    # Trigger option
        data_to_send[4] = tigger_setting_l                      # Trigger value low byte
        data_to_send[5] = tigger_setting_h                      # Trigger value high byte
        data_to_send[6] = self.trigger_channel.currentIndex()   # Trigger channel
        data_to_send[7] = self.time_scale.currentIndex()        # TimeScale
        data_to_send[8] = 1

        self.send_function(data_to_send)

        self.set_is_on_function(self.channel1_range.currentIndex() != len(self.__voltage_ranges) - 1,
                                self.channel2_range.currentIndex() != len(self.__voltage_ranges) - 1,
                                self.channel3_range.currentIndex() != len(self.__current_ranges) - 1)
