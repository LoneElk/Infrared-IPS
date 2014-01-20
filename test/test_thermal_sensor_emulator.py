__author__ = 'jim'

import unittest

import serial

from location_sensors.sensor_emulator.thermal_sensor_emulator import SerialPortEmulator,VirtualMLX90620Hw


class SerialPortEmulatorTests(unittest.TestCase):

    def setUp(self):
        self.testEmulator = SerialPortEmulator()
        self.testString = 'Hello World test!'

    def test_class_set_up(self):
        self.assertIsInstance(self.testEmulator,SerialPortEmulator)

    def test_create_port(self):
        self.testEmulator.create_port()
        self.assertEqual(self.testEmulator.port_active,True)
        self.assertRegexpMatches(self.testEmulator.ttyName,'/dev/ttys00')
        self.testEmulator.close_port()
        self.assertEqual(self.testEmulator.port_active,False)

    def test_with(self):
        with self.testEmulator:
            self.assertEqual(self.testEmulator.port_active,True)
            ret = self.testEmulator.send_string_to_serial(self.testString)
            self.assertEqual(ret,len(self.testString))

        self.assertEqual(self.testEmulator.port_active,False)

    def tearDown(self):
        del(self.testEmulator)


class VirtualSensorTests(unittest.TestCase):

    def setUp(self):
        self.virtual_sensor = VirtualMLX90620Hw()
        self.test_string = 'Hello World test!'
        self.test_data_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/location_sensors/sensor_emulator/rawData.json'
        with open(self.test_data_file,'r') as tempFile:
            self.test_reading = tempFile.readline()

    def test_class_set_up(self):
        self.assertIsInstance(self.virtual_sensor,SerialPortEmulator)

    def test_load_sensor_readings(self):
        with self.assertRaises(IOError):
            self.virtual_sensor.load_sensor_readings()
        self.virtual_sensor.sensorReadingFile = self.test_data_file
        self.virtual_sensor.load_sensor_readings()
        self.assertEqual(len(self.virtual_sensor.sensorReadings),907)

    def test_start_sensor(self):
        with self.assertRaises(ValueError):
            self.virtual_sensor.start_sensor()
        self.virtual_sensor.load_sensor_readings(self.test_data_file)
        with self.assertRaises(IOError):
            self.virtual_sensor.start_sensor()

        tty_name = self.virtual_sensor.create_port()
        ser = serial.Serial(port=tty_name)

        self.virtual_sensor.start_sensor()
        self.assertEqual(self.test_reading,ser.readline())
        self.virtual_sensor.stop_sensor()
        #time.sleep(0.1)
        self.assertTrue(not self.virtual_sensor.sensorThread.isAlive())
        ser.close()

    def tearDown(self):
        del self.virtual_sensor

if __name__=='__main__':
    unittest.main()