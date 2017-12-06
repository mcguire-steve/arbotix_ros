#!/usr/bin/env python

# Copyright (c) 2008-2011 Vanadium Labs LLC. 
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Vanadium Labs LLC nor the names of its 
#     contributors may be used to endorse or promote products derived 
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

## @file arbotix.py Low-level code to control an ArbotiX.

import serial, time, sys, thread

from scorpx import *
from struct import unpack

## @brief This class controls an ArbotiX, USBDynamixel, or similar board through a serial connection.
class ArbotiX:

    ## @brief Constructs an ArbotiX instance and opens the serial connection.
    ##
    ## @param port The name of the serial port to open.
    ## 
    ## @param baud The baud rate to run the port at. 
    ##
    ## @param timeout The timeout to use for the port. When operating over a wireless link, you may need to
    ## increase this.
    ## Slightly large, since the bootloader takes a few seconds to run too
    def __init__(self, port="/dev/ttyUSB0",baud=115200, timeout = 0.5, connectAttempts = 10):
        self._mutex = thread.allocate_lock()
        self._ser = serial.Serial()
        
        self._ser.baudrate = baud
        self._ser.port = port
        self._ser.timeout = timeout
        self._ser.open()

       
        #Wait for reset
        attempts = 0
        while attempts  < connectAttempts:
            print 'Connection attempt %d...' % attempts
            self._ser.write(b'1 1 20 0\r') #exit from machine mode or print the menu
            firstRead = ",".join(self._ser.readlines())
            if firstRead.count("machine interface") > 0:
                break
            attempts += 1
            
        #Put the device into a known state:
        
        #Get the servo list
        self._ser.write(b'8\r')

        #Get the servo count from the first response:
        response = self._ser.readlines()
        #print 'Input:'
        #print response
        servoCountLine = response[0].split(" ")
        servoCount = int(servoCountLine[1])
        print 'Found %d servos!' % servoCount

        self.servoMap = dict()
        responseLine = 2;
        for index in range(0,servoCount):
            mapLine = response[responseLine].split(":");
            self.servoMap[index] = ServoState(mapLine[1])
            responseLine+= 1

        #print 'Servo map:'
        #print self.servoMap

        #Switch to machine mode
        self._ser.write(b'10\r')

        #Consume the response
        self._ser.readline()
        
        #Kick off the service loop
        thread.start_new_thread(self.serviceLoop, ())
        
    def serviceLoop(self):
        cmds = []
        cmds.append(FWCommand(self.servoMap[0].id,SX_NOOP, 0))
        while True:
            self.writeCommands(cmds)
            time.sleep(0.05) #10hz update over the serial

    def writeCommands(self, cmdList):
        if len(cmdList) == 0:
            return
        #print 'Writing %d commands...' % len(cmdList)
        #Write the number of commands first
        self._mutex.acquire()  
        self._ser.write(str(len(cmdList))+' ')

        for cmd in cmdList:
            #print '%d %d %d\n' % (cmd.id, cmd.cmd, cmd.value)
            self._ser.write(str(cmd.id) + ' ')
            self._ser.write(str(cmd.cmd) + ' ')
            self._ser.write(str(cmd.value) + ' ')
       
        self._ser.write(b'\r')
        
        #After we write, read from the port to pick up the status line
        #First char: number of servos that we have status on

        numServos = struct.unpack("<B", self._ser.read(1))

        #print 'reading status for %d servos' % numServos[0]
        for i in range(0,numServos[0]):
            #print 'Updating servo %d' % i
            #This assumes that the mapping is well known for each servo already
            #print 'Record is %d bytes long' % self.servoMap[i].getLength()
            self.servoMap[i].update(self._ser.read(self.servoMap[i].getLength()))
        self._mutex.release()

    def updateState(self):
        cmds = []
        cmds.append(FWCommand(self.servoMap[0].id,SX_NOOP, 0))
        self.writeCommands(cmds)

    def printStatusAll(self):
        self.printStatus(self.servoMap.keys())
        
    def printStatus(self, indexes):
        for i in indexes:
            #print 'state of %s' % i
            state = self.servoMap[int(i)]
            print '%d: id %d: Pos: %d Mov: %d En: %d LED: %d' % (int(i),
                                                                state.id,
                                                                state.position,
                                                                state.moving,
                                                                state.enabled,
                                                                state.led)
            
                                                            
                                                            
    ## @brief Turn on the torque of a servo.
    ##
    ## @param index List of IDs to enable.
    ##
    ## @return The error level.
    def enableTorque(self, indexes):
        cmds = []
        for i in indexes:
            cmds.append(FWCommand(self.servoMap[int(i)].id,SX_SET_ENABLED, 1))
        self.writeCommands(cmds)
        

    ## @brief Turn on the torque of a servo.
    ##
    ## @param index The ID of the device to disable.
    ##
    ## @return The error level.
    def disableTorque(self, indexes):
        cmds = []
        for i in indexes:
            cmds.append(FWCommand(self.servoMap[int(i)].id,SX_SET_ENABLED, 0))
        self.writeCommands(cmds)

    ## @brief Set the status of the LED on a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value 0 to turn the LED off, >0 to turn it on
    ##
    ## @return The error level.
    def setLedOn(self, indexes):
        cmds = []
        for i in indexes:
            cmds.append(FWCommand(self.servoMap[int(i)].id, SX_SET_LED, 1))
        return self.writeCommands(cmds)
    
    def setLedOff(self, indexes):
        cmds = []
        for i in indexes:
            cmds.append(FWCommand(self.servoMap[int(i)].id, SX_SET_LED, 0))
        return self.writeCommands(cmds)
    
    ## @brief Set the position of a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The position to go to in, in servo ticks.
    ##
    ## @return The error level.
    def setPosition(self, indexes, values):
        cmds = []
        for i,j in zip(indexes, range(0,len(indexes))):
            if self.servoMap[int(i)].enabled:
                cmds.append(FWCommand(self.servoMap[int(i)].id, SX_SET_POSITION, values[j]))
        return self.writeCommands(cmds)
    
    ## @brief Set the speed of a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The speed to write.
    ##
    ## @return The error level.
    def setSpeed(self, indexes, values):
        cmds = []
        for i,j in zip(indexes, range(0, len(indexes))):
            cmds.append(FWCommand(self.servoMap[int(i)].id, SX_SET_SPEED, values[j]))
        return self.writeCommands(cmds)
    
    ## @brief Get the position of a servo.
    ##
    ## @param index The index of the servo in the map
    ##
    ## @return The servo position.
    def getPosition(self, index):
        return self.servoMap[index].position
    
    ## @brief Get the speed of a servo.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The servo speed.
    def getSpeed(self, index):
        return self.servoMap[index].speed
        
    ## @brief Get the voltage of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The voltage, in Volts.
    def getVoltage(self, index):
       
        return -1    

    ## @brief Get the temperature of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The temperature, in degrees C.
    def getTemperature(self, index):
       
        return -1

    ## @brief Determine if a device is moving.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return True if servo is moving.
    
    def isMoving(self, index):
        return self.servoMap[index].moving

    def isEnabled(self, index):
        return self.servoMap[index].enabled
    
    def getLoad(self, index):
        return self.servoMap[index].load
    
    
    ###########################################################################
    # Extended ArbotiX Driver

    ## Helper definition for analog and digital access.
    LOW = 0
    ## Helper definition for analog and digital access.
    HIGH = 0xff
    ## Helper definition for analog and digital access.
    INPUT = 0
    ## Helper definition for analog and digital access.
    OUTPUT = 0xff

    # ArbotiX-specific register table
    # We do Model, Version, ID, Baud, just like the AX-12
    ## Register base address for reading digital ports
    REG_DIGITAL_IN0 = 5
    REG_DIGITAL_IN1 = 6
    REG_DIGITAL_IN2 = 7
    REG_DIGITAL_IN3 = 8
    ## Register address for triggering rescan
    REG_RESCAN = 15
    # 16, 17 = RETURN, ALARM
    ## Register address of first analog port (read only).
    ## Each additional port is BASE + index.
    ANA_BASE = 18
    ## Register address of analog servos. Up to 10 servos, each
    ## uses 2 bytes (L, then H), pulse width (0, 1000-2000ms) (Write only)
    SERVO_BASE = 26
    # Address 46 is Moving, just like an AX-12
    REG_DIGITAL_OUT0 = 47

    ## @brief Force the ArbotiX2 to rescan the Dynamixel busses.
    def rescan(self):
        self.write(253, self.REG_RESCAN, [1,])

    ## @brief Get the value of an analog input pin.
    ##
    ## @param index The ID of the pin to read (0 to 7).
    ##
    ## @param leng The number of bytes to read (1 or 2).
    ##
    ## @return 8-bit/16-bit analog value of the pin, -1 if error.
    def getAnalog(self, index, leng=1):
        try:
            val = self.read(253, self.ANA_BASE+int(index), leng)
            return sum(val[i] << (i * 8) for i in range(leng))
        except:
            return -1

    ## @brief Get the value of an digital input pin.
    ##
    ## @param index The ID of the pin to read (0 to 31).
    ##
    ## @return 0 for low, 255 for high, -1 if error.
    def getDigital(self, index):
        try:
            if index < 32:
                x = self.read(253, self.REG_DIGITAL_IN0 + int(index/8), 1)[0]
            else:
                return -1
        except:
            return -1
        if x & (2**(index%8)):
            return 255
        else:
            return 0

    ## @brief Get the value of an digital input pin.
    ##
    ## @param index The ID of the pin to write (0 to 31).
    ##
    ## @param value The value of the port, >0 is high.
    ##
    ## @param direction The direction of the port, >0 is output.
    ##
    ## @return -1 if error.
    def setDigital(self, index, value, direction=0xff):
        if index > 31: return -1
        if value == 0 and direction > 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [1])
        elif value > 0 and direction > 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [3])
        elif value > 0 and direction == 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [2])
        else:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [0])
        return 0

    ## @brief Set the position of a hobby servo.
    ##
    ## @param index The ID of the servo to write (0 to 7).
    ##
    ## @param value The position of the servo in milliseconds (1500-2500). 
    ## A value of 0 disables servo output.
    ##
    ## @return -1 if error.
    def setServo(self, index, value):
        if index > 7: return -1
        if value != 0 and (value < 500 or value > 2500):
            print "ArbotiX Error: Servo value out of range:", value
        else:
            self.write(253, self._SERVO_BASE + 2*index, [value%256, value>>8])
        return 0

