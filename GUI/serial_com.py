import time

from serial import *
import serial.tools.list_ports as serial_ports

class SerialCom(Serial):
    def __init__(self):
        super(SerialCom, self).__init__()
        self.baudrate = 921600
        self.bytesize = EIGHTBITS
        self.parity = PARITY_NONE
        self.stopbits = STOPBITS_ONE
        self.timeout = 2
        self.is_serial_open = False

    def search_serial(self):
        if not self.is_serial_open:
            ports = serial_ports.comports()
            for p in ports:
                if p.vid == 1155 and p.pid == 22336:
                # if p.name == "COM2":
                    self.port = p.name
                    if self.isOpen():
                        self.is_serial_open = True
                        self.reset_input_buffer()
                        self.reset_output_buffer()
                        print("Connected to device on port ", p.name)
                    else:
                        self.open()

        return self.is_serial_open

    def send_settings(self, settings):
        if not self.isOpen():
            return
        self.write(settings)
