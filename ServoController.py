import struct
import serial
import time


# FIXME: magic functions to work with 'with' keyword
class ServoController:
	
    def __init__(self, usb_port='/dev/ttyACM0'):
        self.sc = serial.Serial(usb_port, timeout=1)

    def close(self):
        self.sc.close()

    def setAngle(self, n, angle):
        if angle > 180 or angle <0:
           angle=90
        byteone=int(254*angle/180)
        bud=chr(0xFF)+chr(n)+chr(byteone)
        self.sc.write(bud)
        self.sc.flush()

    def setPosition(self, servo, position):
        position = position * 4
        poslo = (position & 0x7f)
        poshi = (position >> 7) & 0x7f
        chan = servo & 0x7f
        data = chr(0xaa) + chr(0x0c) + chr(0x04) + chr(chan) + chr(poslo) + chr(poshi)
        self.sc.write(data)
        self.sc.flush()

    def setSpeed(self, servo, speed):
        speedlo = (speed & 0x7f)
        speedhi = (speed >> 7) & 0x7f
        chan = servo & 0x7f
        data = chr(0xaa) + chr(0x0c) + chr(0x07) + chr(chan) + chr(speedlo) + chr(speedhi)
        self.sc.write(data)
        self.sc.flush()

    def setAcceleration(self, servo, acceleration):
        accello = (acceleration & 0x7f)
        accelhi = (acceleration >> 7) & 0x7f
        chan = servo & 0x7f
        data = chr(0xaa) + chr(0x0c) + chr(0x09) + chr(chan) + chr(accello) + chr(accelhi)
        self.sc.write(data)
        self.sc.flush()

    def getPosition(self, servo):
        chan = servo & 0x7f
        data = chr(0xaa) + chr(0x0c) + chr(0x10) + chr(chan)
        self.sc.write(data)
        self.sc.flush()
        w1 = ord(self.sc.read())
        w2 = ord(self.sc.read())
        b = chr(w1) + chr(w2)
        return struct.unpack('h',b)[0]/4

    def getErrors(self):
        data = chr(0xaa) + chr(0x0c) + chr(0x21)
        self.sc.write(data)
        w1 = ord(self.sc.read())
        w2 = ord(self.sc.read())
        return w1, w2

