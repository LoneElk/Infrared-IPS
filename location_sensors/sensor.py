#!/usr/bin/env python
#coding=utf-8

"""
This is a sensor class used to collect data from Thermopile Sensors
The class wraps the python serial class to abstract parsing of sensor data.
See W3C Sensor Ontology http://www.w3.org/2005/Incubator/ssn/XGR-ssn-20110628/ and
classes described by Abdullah Al Mamun et al in http://www.dsmforum.org/events/dsm13/Papers/AlMamun.pdf
"""

__author__ = 'Jim Brown'
__version__ = '0.0.1'

import time # standard python time function
import json # Sensor HW produces data is produced as json object
import abc
from pydispatch import dispatcher

import serial # PySerial used to connect to the Arduino board, os and posix used for unit testing

from location_sensors.sensor_properties import MLX90620SensorOutput


# ----------------- Constants -----------------
from globals.global_constants import *


class ReadSensorDataTimeOutException(Exception):
    """Customer exception class to allow the readSensorDataTimeOut to be flagged"""
    pass

#Abstract classes not really adding any value taken out of model

class Sensor(object):
    """Generic Sensor base class holds the sensor type attribute"""
    def __init__(self,arg_sensor_type):
        self.observation_value = None
        self.sensor_type = arg_sensor_type

    @abc.abstractmethod
    def make_observation(self):
        self._broadcast_sensor_output()
        return self.observation_value

    def _broadcast_sensor_output(self):
        dispatcher.send(self.observation_value, self)

class InfraredSensor(Sensor):
    """Generic Infrared sensor class; sets the sensor type attribute to infrared"""
    def __init__(self,arg_sensor_properties):
        super(InfraredSensor,self).__init__(SensorType.infrared)
        self.sensor_properties = arg_sensor_properties

class MLX90620(serial.Serial, InfraredSensor):
    """Sensor class retrieves temperature information from the Sensor connected on the specified Serial Port
    Attributes:
    sensorConfig: Holds config for this sensor including {nodeName, sensorFlush, sensorArraySize, ambientDataPos,
    timeBetweenAmbientTemp,sensorColumns,
    sensorRows, sensorArrayTranspose, sensorPort,baudRate &  timeout} - see SensorConfig class for details.
    """
    def __init__(self,arg_sensor_properties):
        """Initiation of the function works by opening appropriate serial port and setting timeout for the sensor"""
        # noinspection PyArgumentList
        serial.Serial.__init__(self,port=arg_sensor_properties.port,
                               baudrate=arg_sensor_properties.baud_rate,
                               timeout=arg_sensor_properties.time_out)

        InfraredSensor.__init__(self,arg_sensor_properties)

        self.sensor_properties = arg_sensor_properties
        self.observation_value = None
        self._last_raw_sensor_output = ''

    def make_observation(self):
        """The sensor data is continually dumped by the arduino onto the serial port
        the readSensorData method wraps the Serial.readline() function and returns a
        list of float temp sensor data, it checks to ensure that the right number of sensors (the sensorArraysize)
        have been captured before returning"""
        if not self._isOpen: raise serial.portNotOpenError
        record_to_be_found = True
        return_value = ''

        if self.inWaiting() > self.sensor_properties.sensing_array_size*6*2 and self.sensor_properties.flush:
            # Float to 2dp with temp of ~10-50 Degrees results in 6 chr per array field
            # Need to FlushInput Buffer to ensure latest record is picked up,
            # Serial.FlushInput seems to stop Mac Os X reading the Serial Stream
            dump = self.read(self.inWaiting())
            del dump

        time_out = time.time() + self.sensor_properties.time_out
        while record_to_be_found and time.time() < time_out:
            try:
                line = self.readline()
                self._last_raw_sensor_output = line
                line = line.strip('$*\r\n').strip()

                try:
                    temp_reading = json.loads(line)  # Attempt to load retrieved data into temp json object

                    if len(temp_reading[TO]) == self.sensor_properties.sensing_array_size:  # Check array is as per config
                        record_to_be_found = False
                        return_value = temp_reading
                    else:
                        record_to_be_found = True

                except ValueError:
                    record_to_be_found = True

            except serial.SerialTimeoutException:
                pass  # keep trying until readSensorData times out
            except:
                raise serial.SerialException

        if return_value == '':
            raise ReadSensorDataTimeOutException('No data returned from sensor')
        else:
            self.observation_value = MLX90620SensorOutput(return_value,self.sensor_properties)
            self._broadcast_sensor_output()
            return self.observation_value

    def get_latest_ambient_temp(self):
        if self.observation_value is None or \
                (self.observation_value.time_stamp + self.sensor_properties.time_between_ambient_temp <= time.time()):
            self.make_observation()

        return self.observation_value.ambient_data

    def get_latest_observed_temp_shaped(self):
        self.make_observation()
        return self.observation_value.data_shaped

    @property
    def __dict__(self):
        #Serial does not support __dict__
        return dict(InfraredSensor.__dict__.items() + self.sensor_properties.__dict__.items())

    def __str__(self):
        serial_return = (super(serial.Serial, self).__str__())
        return '%s, %s, %s, %s' % (self.sensor_properties.sensor_name, serial_return,
                                   self.sensor_properties,self.observation_value)



