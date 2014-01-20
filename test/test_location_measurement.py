from location_rules_engine.measurement_models.location_measurement import AngularLocationMeasurement

__author__ = 'jim'

import unittest

from numpy import nan,isnan, pi

from location_sensors.sensor_properties import MLX90620SensorProperties


class IMLX90620LocationMeasurementTest(unittest.TestCase):

    def setUp(self):
        self.sensor_properties = MLX90620SensorProperties('Location Measurement Test')
        self.location_measurement = AngularLocationMeasurement(nan, 0, 0,
                                                               self.sensor_properties, self.sensor_properties.name)

    def test_Init(self):
        self.assertTrue(isnan(self.location_measurement.local_position))
        self.assertEqual(self.location_measurement.number_of_objects,0)
        self.assertTrue(isnan(self.location_measurement.global_position))

    def test_AoACalculations(self):

        test_cases = [[0,0],[2*pi,0],[-pi,-pi],[pi/2,pi/2],[-pi/2,-pi/2]]

        for test_case in test_cases:
            self.location_measurement.local_position = test_case[0]
            self.assertEqual(test_case[1],self.location_measurement.local_position)
            self.assertEqual(test_case[1],self.location_measurement.global_position)

        test_cases = [[0,0,pi/2.],[2*pi,0,pi/2.],[-pi,-pi,-pi/2.],[pi/2.,pi/2.,pi],[-pi/2.,-pi/2.,0]]
        self.location_measurement.sensor_properties.location.heading = pi*5/2. # Should be same as pi/2.
        self.location_measurement.sensor_properties.calculate_measurement_boundary()

        for test_case in test_cases:
            self.location_measurement.local_position = test_case[0]
            self.assertEqual(test_case[1],self.location_measurement.local_position)
            self.assertEqual(test_case[2],self.location_measurement.global_position)

        test_cases = [[0,0,-pi/2.],[2*pi,0,-pi/2.],[-pi,-pi,-pi*3/2.],[pi/2.,pi/2.,0],[-pi/2.,-pi/2.,-pi]]
        self.location_measurement.sensor_properties.location.heading = -pi*5/2. # Should be same as -pi/2.
        self.location_measurement.sensor_properties.calculate_measurement_boundary()

        for test_case in test_cases:
            self.location_measurement.local_position = test_case[0]
            self.assertEqual(test_case[1],self.location_measurement.local_position)
            self.assertAlmostEqual(test_case[2],self.location_measurement.global_position,places=5)

        self.location_measurement.sensor_properties.location.heading = -pi*3/2.

if __name__ == '__main__':
    unittest.main()
