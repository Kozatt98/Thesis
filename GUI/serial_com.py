import time

from serial import *
import serial.tools.list_ports as serial_ports
import numpy as np


class SerialCom(Serial):
    def __init__(self):
        super(SerialCom, self).__init__()
        self.baudrate = 921600
        self.bytesize = EIGHTBITS
        self.parity = PARITY_NONE
        self.stopbits = STOPBITS_ONE
        self.timeout = 2
        self.is_serial_open = False
        self.__init_settings__ = np.ones(6, dtype="B")
        self.__init_settings__[0] = 255
        self.__init_settings__[1] = 255
        self.__init_settings__[2] = 255
        self.__init_settings__[3] = 255
        self.__init_settings__[4] = 255
        self.__init_settings__[5] = 255

    def search_serial(self):
        if not self.is_serial_open:
            ports = serial_ports.comports()
            for p in ports:
                # if p.vid == 1155 and p.pid == 22336:
                if p.name == "COM2":
                    self.port = p.name
                    self.open()
                    if self.isOpen():
                        self.write(self.__init_settings__)
                        self.is_serial_open = True
                        self.reset_input_buffer()
                        self.reset_output_buffer()
                        print("Connected to device on port ", p.name)

        return self.is_serial_open

    def send_settings(self, settings):
        self.write(settings)
