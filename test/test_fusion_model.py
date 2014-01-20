from test.observer_class import TestMeasurementObserver

__author__ = 'jim'

import unittest
import time
import sys
import logging

from pydispatch import dispatcher
from numpy import pi
import cv2

from location_rules_engine.fusion_model.angular_measurement_fusion_model import AngularMeasurementFusionModel, \
    AngularFusionModelParams
from location_rules_engine.measurement_models.location_measurement import AngularLocationMeasurement
from globals.position import Position
from location_sensors.sensor_properties import InfraredSensorProperties
from location_rules_engine.measurement_models.measurement_model import MeasurementModel
import location_rules_engine.measurement_models.mlx90620_measurement_model as oim
import globals.geoJson_utility_functions as guf
import globals.global_constants as gc
from globals.building_model import Level


def run_fusion_emulator(time_to_run, pres=100,
                        num_particles=100,
                        sensor_noise=0.05,
                        turn_noise=0.05,
                        forward_noise=0.05,
                        me_speed=0.1,
                        me_turn=0.01,
                        floor_plan=gc.config_floor_plan_file,
                        sensor_config_file=gc.sensor_config_file,
                        sensor_image_scale=50):
    """Start up test emulator and use it to pass data to the Sensor class and then onwards to the measurement model"""
    logging.info('Starting emulation tests')

    try:
        sensor_objects = guf.create_mlx90620_sensors(guf.load_map(sensor_config_file), gc.SensorCreationMode.emulation)
        logging.info('Starting Fusion Model Emulator..')

        measurement_models = []
        for sensor in sensor_objects:
            node_oim = oim.MLX90620MeasurementModel(sensor_objects[sensor]['sensor'])

            node_oim.training_data_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/' + \
                                          'location_rules_engine/measurement_models/training_data/training_data.json'
            node_oim.cv_scale = sensor_image_scale
            node_oim.calibrate_measurement_model()

            measurement_models.append(node_oim)
            #Set up Observer To Display OIM data
            my_test_observer = TestMeasurementObserver(node_oim,sensor )


        floor_plan_geojson_file = floor_plan
        floor_plan = Level(guf.load_map(floor_plan_geojson_file))


        fusion_model_params = AngularFusionModelParams(arg_number_of_particles=num_particles,
                                                       arg_sensor_noise=sensor_noise,
                                                       arg_turn_noise=turn_noise,
                                                       arg_forward_noise=forward_noise,
                                                       arg_speed=me_speed,
                                                       arg_turn=me_turn,
                                                       arg_mean_confidence_threshold=0.95,
                                                       arg_discard_old_measurements=10,
                                                       arg_cvscale=pres)

        fusion_model = AngularMeasurementFusionModel(arg_name='First E2E Test',
                                                     arg_related_measurement_models=measurement_models,
                                                     arg_container=floor_plan.rooms['kitchen'],
                                                     arg_model_params=fusion_model_params)

        for sensor in sensor_objects:
            sensor_objects[sensor]['emulator'].start_sensor()
            sensor_objects[sensor]['sensor'].open()

        timeOut = time.time() + time_to_run

        while time.time() < timeOut:
            try:
                for sensor in sensor_objects:
                   sensor_objects[sensor]['sensor'].make_observation()
            except:
               logging.warning('Measurement failed')

            fusion_model.visualise_measurements()
            time.sleep(0.01)
            cv2.waitKey(1)

        logging.info('closing and tidying up...')

        return_value = 0
    except:
        logging.error("Unexpected error: %s - %s",str(sys.exc_info()[0]), str(sys.exc_info()[1]))
        return_value = 1
    finally:
        for fn in (
            "sensor_objects['node1']['sensor'].close()",
            "sensor_objects['node1']['emulator'].stop_sensor()",
            "sensor_objects['node1']['emulator'].close_port()",
            'cv2.destroyAllWindows()'):
            try:
                exec (fn)
            except:
                logging.warning("clean up error:" + str(sys.exc_info()[0]) + ' - ' + sys.exc_info()[1])
                continue

    return return_value


class TestAngularMeasurementFusionModel(unittest.TestCase):
    def setUp(self):
        # Set up measurement models
        #self.node1_pos = Position(0,0,0)
        #self.node1_prop = sp.MLX90620SensorProperties(arg_name='node1',arg_location=self.node1_pos)
        #self.node1 = sensor.MLX90620(arg_sensor_properties=self.node1_prop)
        #self.node1_oim = mlxmm.MLX90620MeasurementModel(arg_sensor=self.node1)


        #Use floor plan of building - see test floor plan
        self.floor_plan_geojson_file = \
            '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/configuration/floorplan1.json'
        self.floor_plan = Level(guf.load_map(self.floor_plan_geojson_file))


        #Create Generic MeasurementModel as input add sensor properties
        self.node1_mm = MeasurementModel('node1', gc.MeasurementType.angle, None)
        self.node1_mm.sensor_properties = InfraredSensorProperties('node1', Position(0, 0, 0),
                                                                   gc.MeasurementType.temperature, pi / 4., 5, 16)
        self.node2_mm = MeasurementModel('node2', gc.MeasurementType.angle, None)
        self.node2_mm.sensor_properties = InfraredSensorProperties('node2', Position(0, 0, 0),
                                                                   gc.MeasurementType.temperature, pi / 4., 5, 16)
        self.node3_mm = MeasurementModel('node3', gc.MeasurementType.angle, None)
        self.node3_mm.sensor_properties = InfraredSensorProperties('node3', Position(0, 0, 0),
                                                                   gc.MeasurementType.temperature, pi / 4., 5, 16)

        self.model_parameters = AngularFusionModelParams()

        self.fusion_model = \
            AngularMeasurementFusionModel('livingRoom', [self.node1_mm, self.node2_mm],
                                          self.floor_plan.rooms['livingRoom'], self.model_parameters)

    def test_init(self):
        self.assertIsInstance(self.fusion_model, AngularMeasurementFusionModel)
        self.assertTrue(len(dispatcher.senders) == 4)
        self.assertItemsEqual(self.fusion_model.sensor_properties,
                              {self.node1_mm.sensor_properties.name: self.node1_mm.sensor_properties,
                               self.node2_mm.sensor_properties.name: self.node2_mm.sensor_properties})
        self.assertEquals(self.fusion_model.container, self.floor_plan.rooms['livingRoom'])

        self.assertTrue(len(dispatcher.connections) == 4)
        self.fusion_model.subscribe_to_measurement_data_source(self.node3_mm)
        self.assertItemsEqual(self.fusion_model.sensor_properties,
                              {self.node1_mm.sensor_properties.name: self.node1_mm.sensor_properties,
                               self.node2_mm.sensor_properties.name: self.node2_mm.sensor_properties,
                               self.node3_mm.sensor_properties.name: self.node3_mm.sensor_properties})

        self.assertTrue(len(dispatcher.connections) == 5)
        self.fusion_model.disconnect_measurement_data_source(self.node3_mm)
        self.assertItemsEqual(self.fusion_model.sensor_properties,
                              {self.node1_mm.sensor_properties.name: self.node1_mm.sensor_properties,
                               self.node2_mm.sensor_properties.name: self.node2_mm.sensor_properties})
        self.assertTrue(len(dispatcher.connections) == 4)


    def test_create_location_measurement(self):
        self.node1_mm._publish_location_measurement()
        self.node1_mm.location_measurement = AngularLocationMeasurement(0, 1, time.time(),
                                                                        self.node1_mm.sensor_properties,self.node1_mm.name)
        self.node1_mm._publish_location_measurement()
        self.node1_mm._publish_location_measurement()
        self.fusion_model.visualise_measurements()


class TestEmulator(unittest.TestCase):
    def test_object_set_up(self):
        self.assertEquals(0, run_fusion_emulator(1))


if __name__ == '__main__':
    unittest.main()
