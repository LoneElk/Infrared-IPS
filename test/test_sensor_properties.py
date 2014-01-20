__author__ = 'jim'

import unittest
from numpy import pi
from shapely.geometry import Polygon
from location_sensors.sensor_properties import MLX90620SensorProperties
from globals.position import Position

class TestSensorProperties(unittest.TestCase):
    
    def setUp(self):
        self.sensor_properties = MLX90620SensorProperties('Test')

    def test_measurement_boundary(self):
        # Calculated using /Users/jim/Dropbox/Documents/Msc/Thesis/A4/Experiments/sensor_boundry.py
        # base case (0,0,0) range = 1
        self.sensor_properties.range = 1
        self.sensor_properties.fov = pi/3. # Changed this to fit in with the

        self.sensor_properties.location.heading = 0
        expected_results = Polygon([[0, 0], [1, -0.57735026918962562], [1, 0.57735026918962562]])
        self.assertTrue(expected_results.intersects(Polygon(self.sensor_properties.calculate_measurement_boundary()[0])))
        expected_results = [[0.0, 0.0],
                            [0.8650980420904953, -0.4994645874763656],
                            [0.8650980420904953, -0.4994645874763656],
                            [0.8650980420904955, 0.49946458747636563],
                            [0.8819212643483519, 0.4713967368260034],
                            [0.8819212643483553, -0.4713967368259972],
                            [0.9238795325112841, 0.3826834323650961],
                            [0.923879532511287, -0.3826834323650894],
                            [0.9569403357322068, 0.29028467725446927],
                            [0.9569403357322089, -0.2902846772544621],
                            [0.980785280403229, 0.1950903220161357],
                            [0.9807852804032304, -0.19509032201612808],
                            [0.9951847266721962, 0.09801714032956847],
                            [0.9951847266721969, -0.0980171403295605],
                            [1.0, 0.0],
                            [1.0, 8.238535137130597e-15]]

        self.assertItemsEqual(expected_results,sorted(self.sensor_properties.calculate_measurement_boundary()[0]))


        # Rotate Sensor by 90 degrees
        self.sensor_properties.location.heading = pi/2.
        expected_results = Polygon([[0, 0], [-0.57735026918962562, 1], [0.57735026918962562, 1]])
        self.assertTrue(expected_results.intersects(Polygon(self.sensor_properties.calculate_measurement_boundary()[0])))

        # Rotate Sensor by -90 degrees
        self.sensor_properties.location.heading = -pi/2.
        expected_results = Polygon([[0, 0], [-0.57735026918962562, -1], [0.57735026918962562, -1]])
        self.assertTrue(expected_results.intersects(Polygon(self.sensor_properties.calculate_measurement_boundary()[0])))

        # Rotate Sensor by -180 degrees
        self.sensor_properties.location.heading = -pi
        expected_results = Polygon([[0, 0], [-1, -0.57735026918962562], [-1, 0.57735026918962562]])
        self.assertTrue(expected_results.intersects(Polygon(self.sensor_properties.calculate_measurement_boundary()[0])))

        # Move Base Co-ordinate -=-=-=-=-=--=-=-=-=-=--=-=-=-=-=--=-=-=-=-=--=-=-=-=-=--=-=-=-=-=--=-=-=-=-=-
        self.sensor_properties.location = Position(1,1,0)
        expected_results = Polygon([[1, 1], [2, 0.4226497308103744], [2, 1.57735026918962562]])
        self.assertTrue(expected_results.intersects(Polygon(self.sensor_properties.calculate_measurement_boundary()[0])))

        # Move Base Co-ordinate 90 degress
        self.sensor_properties.location.heading = pi/2.
        expected_results = Polygon([[1, 1], [0.4226497308103744, 2], [1.57735026918962562, 2]])
        self.assertTrue(expected_results.intersects(Polygon(self.sensor_properties.calculate_measurement_boundary()[0])))

        # Rotate Sensor by -90 degrees
        self.sensor_properties.location = Position(25,36,0)
        self.sensor_properties.location.heading = -pi/2.
        expected_results = Polygon([[25, 36], [24.422649730810374, 35], [25.57735026918962562, 35]])
        self.assertTrue(expected_results.intersects(Polygon(self.sensor_properties.calculate_measurement_boundary()[0])))


if __name__ == '__main__':
    unittest.main()
