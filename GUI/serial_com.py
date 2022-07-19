from serial import *
import serial.tools.list_ports as serial_ports


class SerialCom(Serial):
    def __init__(self):
        super(SerialCom, self).__init__()
        self.baudrate = 115200
        self.bytesize = EIGHTBITS
        self.parity = PARITY_NONE
        self.stopbits = STOPBITS_ONE
        self.timeout = 1
        self.is_serial_open = False

    def search_serial(self):
        if not self.is_serial_open:
            ports = serial_ports.comports()
            for p in ports:
                print(p.name)
                if (p.vid == 1155 and p.pid == 22336) or p.name == "COM2":
                    self.port = p.name
                    self.open()
                    if self.isOpen():
                        print("Connected to device on port ", p.name)
                        self.is_serial_open = True
        return self.is_serial_open

