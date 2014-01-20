__author__ = 'jim'

import unittest
import logging

import numpy as np

from location_rules_engine.measurement_models.mlx90620_measurement_model import MLX90620MeasurementModel, MLX90620SensorOutput
from location_sensors.sensor_properties import MLX90620SensorProperties
from location_sensors.sensor import MLX90620

# ----------------- Constants -----------------
from globals.global_constants import *


class ObjectDetectionModelTest(unittest.TestCase):
    """Unit Test for Sensor see http://cgoldberg.github.io/python-unittest-tutorial/"""

    def setUp(self):
        #Fixtures

        self.test_sensor_config = MLX90620SensorProperties(arg_name='TestNode')
        self.test_sensor = MLX90620(self.test_sensor_config )
        self.object_id_model = MLX90620MeasurementModel(self.test_sensor)
        self.training_data_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/location_rules_engine/measurement_models/training_data/training_data.json'

    def test_object_set_up(self):
        #Default values
        self.assertEqual(self.object_id_model.min_body_temp, 25)
        self.assertEqual(self.object_id_model.max_body_temp, 35)
        #Check Values can be changed
        self.object_id_model.max_body_temp = 50
        self.assertEqual(self.object_id_model.max_body_temp, 50)

    def test_temperature_bins(self):
        #-ve temperature in large bins
        expected_bins = [(-12.5, -7.5), (-7.5, -2.5), (-2.5, 2.5), (2.5, 7.5), (7.5, 12.5)]
        self.object_id_model.min_temp = -10
        self.object_id_model.max_temp = 10
        self.object_id_model.bin_width = 5
        self.object_id_model._create_temperature_bins()
        self.assertEqual(self.object_id_model.temperature_bins, expected_bins)
        #No Temp Range
        expected_bins = [(-2.5, 2.5)]
        self.object_id_model.min_temp = 0
        self.object_id_model.max_temp = 0
        self.object_id_model.bin_width = 5
        self.object_id_model._create_temperature_bins()
        self.assertEqual(self.object_id_model.temperature_bins, expected_bins)
        #+ve Temp Range Bins width ==  1
        expected_bins = [(-0.5, 0.5), (0.5, 1.5), (1.5, 2.5), (2.5, 3.5), (3.5, 4.5), (4.5, 5.5)]
        self.object_id_model.min_temp = 0
        self.object_id_model.max_temp = 5
        self.object_id_model.bin_width = 1
        self.object_id_model._create_temperature_bins()
        self.assertEqual(self.object_id_model.temperature_bins, expected_bins)
        #+ve Temp Range Bins width <=  1
        expected_bins = [(-0.25, 0.25), (0.25, 0.75), (0.75, 1.25)]
        self.object_id_model.min_temp = 0
        self.object_id_model.max_temp = 1
        self.object_id_model.bin_width = 0.5
        self.object_id_model._create_temperature_bins()
        self.assertEqual(self.object_id_model.temperature_bins, expected_bins)
        #+ve Temp Range Bins width <=  1
        expected_bins = [(-0.125, 0.125), (0.125, 0.375), (0.375, 0.625), (0.625, 0.875), (0.875, 1.125)]
        self.object_id_model.min_temp = 0
        self.object_id_model.max_temp = 1
        self.object_id_model.bin_width = 0.25
        self.object_id_model._create_temperature_bins()
        self.assertEqual(self.object_id_model.temperature_bins, expected_bins)


    def test_train_model(self):

        #--------------------- Check Load from File -----------------
        self.object_id_model.use_training_data_file = True

        # No file provided
        with self.assertRaises(ValueError):
            self.object_id_model.calibrate_measurement_model()

        # File provided doesn't exist --- this causes
        with self.assertRaises(IOError):
            self.object_id_model.training_data_file = 'non-existent-file.json'
            self.object_id_model.calibrate_measurement_model()


        #Data File is Loaded
        self.object_id_model.training_data_file = self.training_data_file

        self.object_id_model.min_temp = 0
        self.object_id_model.max_temp = 2
        self.object_id_model.bin_width = 1
        self.object_id_model.min_body_temp = 1
        self.object_id_model.max_body_temp = 1
        expected_bins = [(-0.5, 0.5), (0.5, 1.5), (1.5, 2.5)]

        #prob_o_bar = 1, away from ambient Temp/Sensor Mean, Never See Object
        self.object_id_model.prob_o_bar = 1
        expected_results = [[(-0.5, 0.5), 0.0], [(0.5, 1.5), 0.0], [(1.5, 2.5), 0.0]]
        self.object_id_model.calibrate_measurement_model()
        self.assertEqual(self.object_id_model.temperature_bins, expected_bins)
        self.object_id_model._calculate_occupancy_probability_array()
        self.assertEqual(self.object_id_model.prob_object_present_given_temp, expected_results)

        #prob_o_bar = 1, away from ambient Temp/Sensor Mean, Always See Object
        self.object_id_model.prob_o_bar = 0
        expected_results = [[(-0.5, 0.5), 0.0], [(0.5, 1.5), 1.0], [(1.5, 2.5), 0.0]]
        self.object_id_model.calibrate_measurement_model()
        self.assertEqual(self.object_id_model.temperature_bins, expected_bins)
        self.object_id_model._calculate_occupancy_probability_array()
        self.assertEqual(self.object_id_model.prob_object_present_given_temp, expected_results)

        #Create poly to check different prob_o_bar Results - derived from original experiments
        #See http://glowingpython.blogspot.co.uk/2011/07/polynomial-curve-fitting.html

        # use the model settings below 19,19,1,19,19
        # probo_t = []
        # for i in np.arange(0, 1.1, 0.1):
        #     object_id_model.prob_o_bar = i
        #     object_id_model.calibrate_sensor_model()
        #     probo_t.append(object_id_model.prob_object_present_given_temp[0][1])
        # z4 = polyfit(np.arange(0, 1.1, 0.1), probo_t, 4)
        # z4 is poly used below

        prob_poly = np.poly1d(np.array([-1.8452217, 2.22848291, -1.25934607, -0.11827212, 0.9976473])
)

        self.object_id_model.min_temp = 19
        self.object_id_model.max_temp = 19
        self.object_id_model.bin_width = 1
        self.object_id_model.min_body_temp = 19
        self.object_id_model.max_body_temp = 19

        for i in np.arange(0, 1.1, 0.1):
            self.object_id_model.prob_o_bar = i
            self.object_id_model.calibrate_measurement_model()
            self.object_id_model._calculate_occupancy_probability_array()
            self.assertAlmostEqual(self.object_id_model.prob_object_present_given_temp[0][1], prob_poly(i), places=1)

    def test_log_odds_initialisation(self):
        self.object_id_model.initialise_log_odds()
        self.assertTrue(np.allclose(self.object_id_model.odds_cells_occupied.data_flatten, np.zeros(64)))
        test_prob = np.empty(64)
        test_prob.fill(0.5)
        self.assertTrue(np.allclose(self.object_id_model.odds_cells_occupied.prob_cells_occupied_flatten, test_prob))
        self.assertEqual(self.object_id_model.odds_cells_occupied.prob_cells_occupied_shaped.shape, (4, 16))
        self.assertEqual(self.object_id_model.odds_cells_occupied.data_shaped.shape, (4, 16))

    def test_calculate_log_odds(self):
        self.object_id_model.training_data_file = self.training_data_file

        #Load Training Data First & calibrate model; 3 buckets, with Body temperature similar to mean
        self.object_id_model.min_temp = 18
        self.object_id_model.max_temp = 20
        self.object_id_model.bin_width = 1
        self.object_id_model.min_body_temp = 19
        self.object_id_model.max_body_temp = 19
        self.object_id_model.calibrate_measurement_model()

        #Create Small array for testing & reshape config
        small_reading_array = {'Ta':19.0, 'To':[19.0]}  # Ambient Temp not making a difference here, should it?

        self.test_sensor.sensor_properties.sensing_array_size = 1
        self.test_sensor.sensor_properties.sensing_array_columns = 1
        self.test_sensor.sensor_properties.sensing_array_rows = 1
        self.object_id_model.cv_scale = 1
        self.object_id_model.initialise_log_odds()

        test_reading = MLX90620SensorOutput(small_reading_array, self.test_sensor.sensor_properties)
        self.object_id_model._calculate_log_odds(test_reading)
        self.assertAlmostEqual(self.object_id_model.odds_cells_occupied.data_flatten.tolist()[0], 1.3133,
                               places=2)

        self.object_id_model.visualise_model_data()

    def test_calculate_prob_object_online(self):
        self.object_id_model.training_data_file = self.training_data_file
        #Load Training Data First & calibrate model; 3 buckets, with Body temperature similar to mean
        self.object_id_model.min_temp = 18
        self.object_id_model.max_temp = 20
        self.object_id_model.bin_width = 1
        self.object_id_model.min_body_temp = 19
        self.object_id_model.max_body_temp = 19
        self.object_id_model.calibrate_measurement_model()
        small_reading_array = {'Ta':19.0, 'To':[19.0]}  # Ambient Temp not making a difference here, should it?
        prob_cell_occupied = self.object_id_model._calculate_occupancy_probability(small_reading_array['To'])
        self.assertAlmostEqual(prob_cell_occupied, 0.79, places=2)


    def test_find_objects(self):
        reading_array = {TA: 23.52, TO:[22.07, 21.65, 21.61, 20.79, 21.21, 21.39, 20.92, 20.0, 21.47, 19.86, 20.76, 21.39, 20.25, 21.83,
                        21.39, 20.09, 20.5, 19.98, 21.16, 20.83, 21.05, 20.57,
                        21.6, 20.13, 23.78, 23.43, 22.05, 22.23, 26.34, 25.45, 24.82, 24.21, 26.34, 25.78, 25.47, 24.58,
                        26.02, 25.48, 26.14, 25.72, 25.69, 26.2, 25.54, 24.99,
                        25.72, 25.2, 25.24, 24.63, 24.64, 25.25, 25.66, 25.48, 20.87, 20.79, 24.57, 25.12, 20.29, 19.73,
                        20.2, 21.68, 21.02, 20.35, 21.71, 21.07]}

        self.object_id_model.training_data_file = self.training_data_file
        self.object_id_model.calibrate_measurement_model()
        self.object_id_model.initialise_log_odds()
        test_reading = MLX90620SensorOutput(reading_array, self.test_sensor.sensor_properties)

        self.assertEqual(self.object_id_model._number_of_objects, 0)
        self.object_id_model.cv_scale = 1
        for i in range(0, 2):
            self.object_id_model.create_location_measurement(test_reading)

        #self.assertAlmostEqual(self.object_id_model.aoa, 82.5,places=1)

        for i in range(0, 3):
            self.object_id_model.create_location_measurement(test_reading)
        self.assertEqual(self.object_id_model.time_stamp, test_reading.time_stamp)
        self.assertEqual(self.object_id_model._number_of_objects, 1)

        logging.debug("Tracking info cv_scale: {0}, fov: {1}, tracking circle: {2}".format(
            self.object_id_model.cv_scale,
            self.object_id_model.sensor_properties.fov,
            self.object_id_model.tracking_circle))

        self.assertAlmostEqual(self.object_id_model._aoa, -0.13089969389957468, places=1)

        self.object_id_model.initialise_log_odds()
        self.object_id_model.cv_scale = 1
        for i in range(0, 3):
            self.object_id_model.create_location_measurement(test_reading)

        self.assertAlmostEqual(self.object_id_model._aoa, -0.13089969389957468, places=1)


    def test_update_temp_stats(self):

        #------------------------ Selective Updates --------------------------------
        # Only cell 64 has object present - it is excluded from the temp stats

        reading_array = {TA: 23.52, TO: [18.0, 18.0, 18.0,  18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,
                                          18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,  18.0, 18.0, 18.0,
                                          18.0, 18.0, 18.0, 18.0, 18.0, 18.0,  18.0, 18.0, 18.0, 18.0, 18.0,  18.0, 18.0,
                                          18.0, 18.0, 18.0,  18.0, 18.0, 18.0, 18.0, 18.0, 18.0,  18.0, 18.0, 18.0, 18.0,
                                          18.0,  18.0, 18.0, 18.0, 18.0, 18.0,  18.0, 18.0, 18.0, 18.0, 18.0, 30.0]}

        test_reading = MLX90620SensorOutput(reading_array, self.test_sensor.sensor_properties)

        odds_array = {TA: 0.0,
                     TO: [-5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
                           -5.0, -5.0, -5.0,
                           -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
                           -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
                           -5.0, -5.0, -5.0,
                           -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, 5.0]}

        self.object_id_model.initialise_log_odds()
        self.object_id_model.odds_cells_occupied.raw_sensor_data = odds_array

        self.object_id_model._update_temp_stats(test_reading,False)

        self.assertEqual(self.object_id_model._sum_object_temp1,sum(reading_array[TO][:63]))
        self.assertEqual(self.object_id_model._count_object_temp,len(reading_array[TO][:63]))
        self.assertEqual(self.object_id_model._sensor_temp_mean,np.mean(reading_array[TO][:63]))
        self.assertAlmostEqual(self.object_id_model._sensor_temp_std_dev,np.std(reading_array[TO][:63]),6)


        # counters remain unchanged when no cells require updating

        odds_array = {'Ta': 0.0,
                     'To': [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                            5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                            5.0, 5.0, 5.0, 5.0,
                            5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                            5.0, 5.0, 5.0, 5.0, 5.0, 5.0]}

        self.object_id_model.odds_cells_occupied.raw_sensor_data = odds_array
        self.object_id_model._update_temp_stats(test_reading,False)

        self.assertEqual(self.object_id_model._sum_object_temp1,sum(reading_array[TO][:63]))
        self.assertEqual(self.object_id_model._count_object_temp,len(reading_array[TO][:63]))
        self.assertEqual(self.object_id_model._sensor_temp_mean,np.mean(reading_array[TO][:63]))
        self.assertAlmostEqual(self.object_id_model._sensor_temp_std_dev,np.std(reading_array[TO][:63]),6)


        #----------------- Load Sensor Data directly ---------------------------------------

        self.object_id_model._initialise_temp_counters()

        reading_array = {TA: 23.52, TO:[22.07, 21.65, 21.61, 20.79, 21.21, 21.39, 20.92, 20.0, 21.47, 19.86, 20.76, 21.39, 20.25, 21.83,
                        21.39, 20.09, 20.5, 19.98, 21.16, 20.83, 21.05, 20.57,
                        21.6, 20.13, 23.78, 23.43, 22.05, 22.23, 26.34, 25.45, 24.82, 24.21, 26.34, 25.78, 25.47, 24.58,
                        26.02, 25.48, 26.14, 25.72, 25.69, 26.2, 25.54, 24.99,
                        25.72, 25.2, 25.24, 24.63, 24.64, 25.25, 25.66, 25.48, 20.87, 20.79, 24.57, 25.12, 20.29, 19.73,
                        20.2, 21.68, 21.02, 20.35, 21.71, 21.07]}
        test_reading = MLX90620SensorOutput(reading_array, self.test_sensor.sensor_properties)

        self.object_id_model._update_temp_stats(test_reading)
        self.assertEqual(self.object_id_model._sum_object_temp1,sum(reading_array[TO]))
        self.assertEqual(self.object_id_model._count_object_temp,len(reading_array[TO]))
        self.assertAlmostEqual(self.object_id_model._sensor_temp_mean,np.mean(reading_array[TO]),6)
        self.assertAlmostEqual(self.object_id_model._sensor_temp_std_dev,np.std(reading_array[TO]),6)

        #----------------- From Sensor Data Array ------------------

        self.object_id_model.use_training_data_file = False
        test_temp = [{TA:22.73, TO:[21.81, 23.60, 23.04, 21.81, 22.49, 22.51, 22.02, 22.48, 21.56, 21.65, 21.65, 21.51, 22.08, 22.18, 22.14,
             21.58, 22.54, 22.21, 22.23, 22.54, 21.77, 21.88, 21.86, 22.58, 21.03, 21.55, 21.21, 21.78, 21.08, 21.57,
             21.57, 21.44, 21.46, 22.63, 21.94, 22.61, 21.48, 21.93, 21.93, 22.22, 21.46, 21.92, 21.91, 21.02, 21.44,
             21.90, 21.15, 21.39, 20.55, 21.88, 21.86, 22.18, 21.75, 20.71, 21.07, 20.87, 21.70, 20.96, 20.17, 21.31,
             21.62, 19.94, 20.43, 20.69]}]

        self.object_id_model.raw_training_data = test_temp
        self.assertAlmostEqual(22.7, self.object_id_model.mean_ambient_temp, places=1)
        self.assertAlmostEqual(0.7, self.object_id_model.sensor_temp_std_dev, places=1)

        self.assertEqual(sum(test_temp[0][TO]),self.object_id_model._sum_object_temp1)
        self.assertEqual(len(test_temp[0][TO]), self.object_id_model._count_object_temp)


        #---------------- Check Update of Sensor Stats as part of ongoing processing ------

        self.object_id_model.calibrate_measurement_model()
        self.object_id_model.create_location_measurement(test_reading)

        #Check stats updated
        self.assertAlmostEqual(np.mean([test_temp[0][TA], reading_array[TA]]),
                               self.object_id_model.mean_ambient_temp, places=1)

        test_calc_array = test_temp[0][TO] + [j for i, j in
                                               zip(self.object_id_model.odds_cells_occupied.prob_cells_occupied_flatten.tolist(),
                                                   reading_array[TO]) if
                                               i <= 1 - self.object_id_model.object_prob_threshold]

        self.assertAlmostEqual(np.mean(test_calc_array), self.object_id_model.sensor_temp_mean, places=2)
        self.assertAlmostEqual(np.std(test_calc_array), self.object_id_model.sensor_temp_std_dev, places=2)

        self.object_id_model.create_location_measurement(test_reading)

        test_calc_array += [j for i, j in
                            zip(self.object_id_model.odds_cells_occupied.prob_cells_occupied_flatten.tolist(), reading_array[TO])
                            if i <= 1 - self.object_id_model.object_prob_threshold]

        self.assertAlmostEqual(np.mean([test_temp[0][TA], reading_array[TA],reading_array[TA]]), self.object_id_model.mean_ambient_temp, places=2)
        self.assertAlmostEqual(np.mean(test_calc_array), self.object_id_model.sensor_temp_mean, places=2)
        self.assertAlmostEqual(np.std(test_calc_array), self.object_id_model.sensor_temp_std_dev, places=2)

        #----------- Load from File --------------
        self.object_id_model.use_training_data_file = True

        # No file provided
        with self.assertRaises(ValueError):
            self.object_id_model.calibrate_measurement_model()

        #meanTemp remains as per existing calculations see Load from Array Above
        self.assertAlmostEqual(np.mean([test_temp[0][TA], reading_array[TA],reading_array[TA]]), self.object_id_model.mean_ambient_temp, places=2)
        self.assertAlmostEqual(np.mean(test_calc_array), self.object_id_model.sensor_temp_mean, places=2)
        self.assertAlmostEqual(np.std(test_calc_array), self.object_id_model.sensor_temp_std_dev, places=2)

        #Data File is Loaded
        self.object_id_model.training_data_file = self.training_data_file

        #Check Key Sensor Stats
        self.assertAlmostEqual(22.3, self.object_id_model.mean_ambient_temp, places=1)

        #Values calculated from training file using np.cusum as follows
        # tempjson = [ json.loads(i) for i in tempfile.readlines()]
        # stream = array([ i['To'] for i in tempjson]).flatten()
        # stream.cumsum()
        # stream.cumsum()[-1] / len(stream)
        # stream2 = power(stream,2)
        # stream2.cumsum()[-1]/len(stream)
        # power(stream2.cumsum()[-1]/len(stream) - power(stream.cumsum()[-1] / len(stream),2),1/2.)

        self.assertAlmostEqual(206010.61,self.object_id_model._sum_object_temp1,places=2)
        self.assertEqual(10752, self.object_id_model._count_object_temp)

        self.assertAlmostEqual(1.4, self.object_id_model.sensor_temp_std_dev, places=1)
        self.assertAlmostEqual(19.2, self.object_id_model.sensor_temp_mean, places=1)

        #---------------- Check Update of Sensor Stats as part of ongoing processing ------

        new_ambient = (len(self.object_id_model.training_data) * self.object_id_model.mean_ambient_temp +
                       reading_array[TA]) / (len(self.object_id_model.training_data) + 1)

        new_test_array = np.array([i.data_flatten
                                   for i in self.object_id_model.training_data]).flatten().tolist() + \
                         [j for i, j in
                          zip(self.object_id_model.odds_cells_occupied.prob_cells_occupied_flatten.tolist(),
                              reading_array[TO])
                          if i <= 1 - self.object_id_model.object_prob_threshold]

        self.object_id_model.calibrate_measurement_model()
        self.object_id_model.create_location_measurement(test_reading)


        #Check stats updated

        self.assertAlmostEqual(new_ambient, self.object_id_model.mean_ambient_temp, places=1)
        self.assertAlmostEqual(np.mean(new_test_array), self.object_id_model.sensor_temp_mean, places=2)
        self.assertAlmostEqual(np.std(new_test_array), self.object_id_model.sensor_temp_std_dev, places=2)

    def tearDown(self):
        pass

if __name__ == '__main__':
    unittest.main()
