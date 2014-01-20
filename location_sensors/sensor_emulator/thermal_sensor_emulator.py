#!/usr/bin/env python

"""
Utility to emulate a sensor serial port, the virtual sensor class 
"""
import posix
import os
import threading
import time
import logging

#----------------------------GLOBALS--------------------------------------------------------
SENSOR_DATA_SPEED = 0.1 # Speed at which data is collected from MLX90620


class SerialPortEmulator(object):
    '''Class wraps posix and os commands to create virtual serial port'''
    
    @property
    def port_active(self):
        return os.isatty(self.fdSlave)
    
    def __init__(self):
        self.fdMaster = -1
        self.fdSlave = -1
        self.ttyName = ''
    def create_port(self):
        self.fdMaster, self.fdSlave = posix.openpty()
        self.ttyName = os.ttyname(self.fdSlave)
        return self.ttyName
        
    def close_port(self):
        os.close(self.fdSlave)
        os.close(self.fdMaster)
    
    def send_string_to_serial(self,argString):
        returnValue = os.write(self.fdMaster,argString)
        return returnValue
    
    def __enter__(self):
        self.create_port()
        
    def __exit__(self, type, value, tb):
        self.close_port()
    

class VirtualMLX90620Hw(SerialPortEmulator):
    '''Class creates a virtual serial port, and uses a file with stored readings from a sensor to produce a virtual
    MLX90620 device to be used to test upstream applications'''
    def __init__(self,argSensorReadingsFile=''):
        super(VirtualMLX90620Hw,self).__init__()
        
        #Set up sensor data
        self.sensorReadingFile = argSensorReadingsFile
        self.sensorReadings = []
        
        #Threading used to allow consuming tasks to be completed in the background.
        self.stopEvent = threading.Event()
        self.stopEvent.clear()
        self.sensorThread= threading.Thread(target=self._write_sensor_readings,args=(self.stopEvent,)) #note tuple (x,) needed
        #Ensure the process terminates with the class.
        self.sensorThread.daemon = True
        
    def load_sensor_readings(self,argSensorReadingsFile=''):
        #Set up sensor readings data
        if argSensorReadingsFile !='':
            self.sensorReadingFile = argSensorReadingsFile
        with open(self.sensorReadingFile,'r') as sensorFile:
            self.sensorReadings = sensorFile.readlines()
    
    def start_sensor(self):
        #Check we've got data loaded and an active port to send data to
        if self.sensorReadings == []:
            raise ValueError('No Sensor Readings')
        elif self.port_active == False:
            raise  IOError('No Active Port')
        #Start sensor thread
        self.sensorThread.start()
    
    def stop_sensor(self):
        # Solution from  http://stackoverflow.com/questions/6524459/stopping-a-thread-python
        self.stopEvent.set()
        time.sleep(0.1) #If the main process is not paused here find that the thread doesn't close
         
    def _write_sensor_readings(self,stopEvent):
        # Loop through process data and print to Serial Port
        line = 0 
        while (not stopEvent.is_set()):
            logging.debug('%s - Wrote sensor reading to wire:',self.ttyName)
            self.send_string_to_serial(self.sensorReadings[line])
            line = (line + 1) % len(self.sensorReadings)
            stopEvent.wait(SENSOR_DATA_SPEED) #without this pause thread doesn't end on stopEvent being fired
