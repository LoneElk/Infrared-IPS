__author__ = 'jim'

import unittest
import posix
import os
import time
import logging

import numpy as np

from location_sensors.sensor import MLX90620, ReadSensorDataTimeOutException
from location_sensors.sensor_properties import MLX90620SensorProperties



# ----------------- Constants -----------------
from globals.global_constants import *


class SensorTest(unittest.TestCase):
    """Unit Test for Sensor see http://cgoldberg.github.io/python-unittest-tutorial/"""

    def setUp(self):
        #Fixtures
        self.test_temp_str = '{"Ta":22.51, "To":[20.27,20.50,19.27,19.62,20.07,19.25,20.29,19.98,20.83,19.63,19.18,' \
                           '19.76,20.49,20.74,19.87,19.92,19.80,19.26,20.46,19.71,19.93,19.47,19.39,19.44,19.62,' \
                           '20.96,21.69,21.14,26.39,24.74,25.16,26.20,26.75,26.74,26.76,26.91,26.09,27.12,26.79,' \
                           '26.20,27.24,26.20,25.89,26.26,26.94,26.94,24.92,25.96,26.77,26.72,25.36,25.26,26.54,' \
                           '26.45,26.14,26.58,23.69,26.38,26.71,26.82,20.41,21.05,21.07,22.82]}\r\n'

        self.test_temp = [20.27, 20.50, 19.27, 19.62, 20.07, 19.25, 20.29, 19.98, 20.83, 19.63, 19.18, 19.76, 20.49,
                         20.74, 19.87, 19.92, 19.80, 19.26, 20.46, 19.71, 19.93, 19.47, 19.39, 19.44, 19.62, 20.96,
                         21.69, 21.14, 26.39, 24.74, 25.16, 26.20, 26.75, 26.74, 26.76, 26.91, 26.09, 27.12, 26.79,
                         26.20, 27.24, 26.20, 25.89, 26.26, 26.94, 26.94, 24.92, 25.96, 26.77, 26.72, 25.36, 25.26,
                         26.54, 26.45, 26.14, 26.58, 23.69, 26.38, 26.71, 26.82, 20.41, 21.05, 21.07, 22.82]
        self.test_ambient_temp = 22.51

        self.test_temp_str2 = '{"Ta":30.73, "To":[30.81,23.60,23.04,21.81,22.49,22.51,22.02,22.48,21.56,21.65,' \
                            '21.65,21.51,22.08,22.18,22.14,21.58,22.54,22.21,22.23,22.54,21.77,21.88,21.86,' \
                            '22.58,21.03,21.55,21.21,21.78,21.08,21.57,21.57,21.44,21.46,22.63,21.94,22.61,' \
                            '21.48,21.93,21.93,22.22,21.46,21.92,21.91,21.02,21.44,21.90,21.15,21.39,20.55,' \
                            '21.88,21.86,22.18,21.75,20.71,21.07,20.87,21.70,20.96,20.17,21.31,21.62,19.94,' \
                            '20.43,20.69]}\r\n'

        self.test_ambient_temp2 = 30.73

        #Use posix.openpty to create dummy Serial Port for Testing
        #http://stackoverflow.com/questions/2174071/how-to-use-dev-ptmx-for-create-a-virtual-serial-port
        self.fd_master, self.fd_slave = posix.openpty()
        self.test_sensor_config = MLX90620SensorProperties(arg_name='TestNode',
                                                           arg_port=os.ttyname(self.fd_slave),
                                                           arg_time_between_ambient_temp=0.2)
        #print self.test_sensor_config.__dict__
        self.test_sensor = MLX90620(self.test_sensor_config)
        #print self.test_sensor.__dict__

    def test_timeOut(self):
        #Check Timeout Error Raised
        with self.assertRaises(ReadSensorDataTimeOutException):
            self.test_sensor.make_observation()

    def test_stringRetival(self):
        #Push test data onto virtual serial port & check retrieved as expected
        os.write(self.fd_master, self.test_temp_str)
        self.assertEqual(self.test_sensor.make_observation().raw_sensor_data[TO], self.test_temp)
        self.assertEqual(self.test_sensor._last_raw_sensor_output, self.test_temp_str)
        #Push test data + partial line onto virtual serial port & check retrieved as expected
        os.write(self.fd_master, self.test_temp_str[len(self.test_temp_str) / 2:] + self.test_temp_str)
        self.assertEqual(self.test_sensor.make_observation().raw_sensor_data[TO], self.test_temp)
        self.assertEqual(self.test_sensor._last_raw_sensor_output, self.test_temp_str)

    def test_ambientData(self):
        os.write(self.fd_master, self.test_temp_str)
        self.assertEqual(self.test_sensor.get_latest_ambient_temp(), self.test_ambient_temp)
        logging.debug('waiting 0.3 seconds')
        time.sleep(0.3)
        os.write(self.fd_master, self.test_temp_str2)
        self.assertEqual(self.test_sensor.get_latest_ambient_temp(), self.test_ambient_temp2)

    def test_SensorTemp(self):
        #Default case where MLX90620 is used with first 4 cells in array representing 1st column of thermopiles
        os.write(self.fd_master, self.test_temp_str)
        self.assertTrue(
            np.allclose(np.array(self.test_temp[:64]).reshape(16, 4).T, self.test_sensor.get_latest_observed_temp_shaped()))
        #Sensor configured so it has all first row sensors in pos 0:16 in array etc..; no transposition
        os.write(self.fd_master, self.test_temp_str)
        self.test_sensor_config.sensing_array_columns = 16
        self.test_sensor_config.sensing_array_rows = 4
        self.test_sensor_config.sensing_array_transpose = False
        self.assertTrue(
            np.allclose(np.array(self.test_temp[:64]).reshape(4, 16), self.test_sensor.get_latest_observed_temp_shaped()))
        #Ambient Sensor in different position
        self.test_sensor_config.ambient_data_pos = 0
        os.write(self.fd_master, self.test_temp_str)
        self.assertTrue(np.allclose(np.array(self.test_temp).reshape(4, 16), self.test_sensor.get_latest_observed_temp_shaped()))

    def tearDown(self):
        self.test_sensor.close()
        os.close(self.fd_slave)
        os.close(self.fd_master)


if __name__ == '__main__':
    unittest.main()
