from serial import *
import serial.tools.list_ports as serial_ports
import numpy as np


class SerialCom(Serial):
    def __init__(self):
        super(SerialCom, self).__init__()
        self.baudrate = 115200
        self.bytesize = EIGHTBITS
        self.parity = PARITY_NONE
        self.stopbits = STOPBITS_ONE
        self.timeout = 5
        self.is_serial_open = False
        self.__init_settings__ = np.ones(3, dtype="B")
        self.__init_settings__[0] = 255
        self.__init_settings__[1] = 255
        self.__init_settings__[2] = 255


    def search_serial(self):
        if not self.is_serial_open:
            ports = serial_ports.comports()
            for p in ports:
                # if p.vid == 1155 and p.pid == 22336:
                if p.name == "COM2":
                    self.port = p.name
                    self.open()
                    if self.isOpen():
                        print("Connected to device on port ", p.name)
                        self.write(self.__init_settings__)
                        self.is_serial_open = True
        return self.is_serial_open

