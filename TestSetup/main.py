import math
import sys
import random
import serial
import time

if __name__ == '__main__':
    ser = serial.Serial()
    ser.port = "COM1"
    ser.baudrate = 3000000
    ser.open()
    ser.timeout = 0.001
    length = 2047
    bits = 8
    while True:
        prefix = [int(0).to_bytes(int(math.ceil(bits / 8)), sys.byteorder),
                  int(1).to_bytes(int(math.ceil(bits / 8)), sys.byteorder),
                  int(2).to_bytes(int(math.ceil(bits / 8)), sys.byteorder)]
        random.shuffle(prefix)
        dataSin = [prefix[0]]
        dataCos = [prefix[1]]
        dataInvSin = [prefix[2]]

        for i in range(length):
            byteSin = (int(((math.sin(2 * math.pi * i / length) + 1) / 2) * (math.pow(2, bits) - 1))) \
                .to_bytes(int(math.ceil(bits / 8)), sys.byteorder)
            byteCos = (int(((math.cos(2 * math.pi * i / length) + 1) / 2) * (math.pow(2, bits) - 1))) \
                .to_bytes(int(math.ceil(bits / 8)), sys.byteorder)
            byteInvSin = (int(((math.cos(2 * math.pi * i / length) * -1 + 1) / 2) * (math.pow(2, bits) - 1))) \
                .to_bytes(int(math.ceil(bits / 8)), sys.byteorder)

            dataSin.append(byteSin)
            dataCos.append(byteCos)
            dataInvSin.append(byteInvSin)

        data = [dataSin, dataCos, dataInvSin]

        data[0] = b''.join(dataSin)
        data[1] = b''.join(dataCos)
        data[2] = b''.join(dataInvSin)

        for i in range(3):
            ser.write(data[i])
        incoming = ser.read(1)
        if incoming != bytes():
            print(incoming)
        time.sleep(0.2)
