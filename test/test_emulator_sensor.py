""" Module to test the processing of data from the Sensor Layer through to the Measurement Layer"""

import time
import unittest
import sys
import logging

import cv2

import location_rules_engine.measurement_models.mlx90620_measurement_model as oim
import globals.geoJson_utility_functions as guf
import globals.global_constants as gc
from test.observer_class import TestMeasurementObserver


def run_test_emulator(time_to_run):
    """Start up test emulator and use it to pass data to the Sensor class and then onwards to the measurement model"""
    logging.info('Starting emulation test.......')

    try:

        sensor_objects = guf.create_mlx90620_sensors(
            guf.load_map(
                '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/configuration/single_sensor_config.json'),
            gc.SensorCreationMode.emulation)

        sensor_objects['node4']['emulator'].start_sensor()
        logging.info('..Connecting Node 4')

        logging.info('..begin processing')

        node4oim = oim.MLX90620MeasurementModel(sensor_objects['node4']['sensor'])

        node4oim.training_data_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/' + \
                                      'location_rules_engine/measurement_models/training_data/training_data.json'
        node4oim.cv_scale = 50
        node4oim.calibrate_measurement_model()

        #Set up Observer To Display OIM data
        my_test_observer = TestMeasurementObserver(node4oim, 'obs2')

        sensor_objects['node4']['sensor'].open()
        timeOut = time.time() + time_to_run

        while time.time() < timeOut:
            try:
                sensor_objects['node4']['sensor'].make_observation()
                logging.debug("Prob obar %s", node4oim.prob_o_bar)
                cv2.imshow('raw image', sensor_objects['node4']['sensor'].observation_value.data_image_array)
            except:
                logging.warning("No sensor data received")

            time.sleep(0.01)
            cv2.waitKey(1)

        logging.info('closing and tidying up...')

        return_value = 0

    except:
        logging.error("Unexpected error: %s, %s" ,str(sys.exc_info()[0]),str(sys.exc_info()[1]))
        return_value = 1
    finally:
        for fn in (
            "sensor_objects['node4']['sensor'].close()",
            "sensor_objects['node4']['emulator'].stop_sensor()",
            "sensor_objects['node4']['emulator'].close_port()",
            'cv2.destroyAllWindows()'):
            try:
                exec (fn)
            except:
                logging.warning("clean up error:", sys.exc_info()[0], sys.exc_info()[1])
                continue

    return return_value


class TestEmulator(unittest.TestCase):
    def test_object_set_up(self):
        self.assertEquals(0, run_test_emulator(1))


if __name__ == '__main__':
    unittest.main()

