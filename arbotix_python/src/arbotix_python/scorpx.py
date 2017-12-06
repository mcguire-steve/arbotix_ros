#!/usr/bin/env python
SX_NOOP=0
SX_SET_POSITION=1
SX_SET_SPEED=2
SX_SET_ENABLED=3
SX_SET_LED=4

#Firmware command interface:
import struct

class FWCommand:
    def __init__(self, _id=1, _cmd=0, _value=0):
        self.id = int(_id)
        self.cmd = int(_cmd)
        self.value = int(_value)

class ServoState:
    def __init__(self, _id = 1):
        self.id = int(_id)
        self.led = 0
        self.enabled = 0
        self.moving = 0
        self.position = 0
        self.speed = 0
        self.load = 0
        self.structFmt = "<BBBBHHH"
        self.structLen = struct.calcsize(self.structFmt)
        
    #Unpack a byte sequence into native Python types
    def update(self, frame):
        fields = struct.unpack(self.structFmt, frame)
        self.id = fields[0]
        self.led = fields[1]
        self.enabled = fields[2]
        self.moving = fields[3]
        self.position = fields[4]
        self.speed = fields[5]
        self.load = fields[6]

    def getLength(self):
        return self.structLen
