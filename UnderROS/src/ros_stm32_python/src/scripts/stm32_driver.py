#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import os, time
import thread


from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

class Stm32:
    ''' Configuration Parameters
    '''

    def __init__(self, port= "/dev/ttyUSB0", baudrate= 115200, timeout= 0.5):
        
        self.port = port 
        self.baudrate = baudrate 
        self.timeout = timeout 
        self.writeTimeout = timeout 
        self.interCharTimeout = timeout / 30.

    def connect(self):
#        try:
            print "Connecting to Stm32 on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
#            # The next line is necessary to give the firmware time to wake up.
#            time.sleep(1)
#            test = self.get_baud()
#            if test != self.baudrate:
#                time.sleep(1)
#                test = self.get_baud()
#                if test != self.baudrate:
#                    raise SerialException
#            print "Connected at", self.baudrate
#            print "Arduino is ready."

#        except SerialException:
#            print "Serial Exception:"
#            print sys.exc_info()
#            print "Traceback follows:"
#            traceback.print_exc(file=sys.stdout)
#            print "Cannot connect to Stm32"
#            os._exit(1)

    def open(self):
        ''' Open the serial port
        '''
        self.port.open()

    def close(self):
        ''' Close the serial port
        '''
        self.port.close()
       
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return int(self.execute('b'));

    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)
if __name__ == "__main__":
    if os.name == "poisix":
        portName = "/dev/ttyACM0"
    else:
        portName = "/dev/ttyUSB0"

    baudRate = 115200

    myStm32 = Stm32(port=portName, baudrate=baudRate, timeout= 0.5)
    myStm32.connect()
#    myStm32.open()
    myStm32.send("VU NGOC SON MUON NAM")
   # myStm32.connect()

    print "Sleeping for 1 second..."
    time.sleep(1) 

    print "Connection test successful.",
    
   # myStm32.stop()
   # myStm32.close()
    
    print "Shutting down Stm32."
