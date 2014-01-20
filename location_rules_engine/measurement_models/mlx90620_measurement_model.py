#!/usr/bin/env python
#coding=utf-8

"""The Object tracking determines probability of Sensor Cell being occupied by a person/object"""
from location_rules_engine.measurement_models.location_measurement import AngularLocationMeasurement

__author__ = 'Jim Brown'
__version__ = '0.0.1'

import json

import numpy as np  # Numpy used for matrix shaping in this module
from scipy import ndimage, stats  # Used for producing CDF and PDF functions for gaussian/normal distribution
from pylab import cm, figure
import cv2

from location_sensors.sensor_properties import MLX90620SensorOutput
from location_rules_engine.measurement_models.measurement_model import MeasurementModel

# ----------------- Constants -----------------
from globals.global_constants import *

# ----------------- Classes --------------------

class OccupancyOdds(MLX90620SensorOutput):
    """Class wraps SensorData and provides addition methods for determining the probability a cell is occupied based"""

    @property
    def prob_cells_occupied_flatten(self):
        return np.array(np.exp(self.data_flatten) / (np.exp(self.data_flatten) + 1))

    @property
    def prob_cells_occupied_shaped(self):
        return self._shape_sensor_data(self.prob_cells_occupied_flatten)


class ObjectIdentificationException(Exception):
    """Customer exception class to allow the probability calculations to be flagged"""
    pass

class MLX90620MeasurementModel(MeasurementModel):
    """Class used to calculate the probability that person is present, estimate the center of mass wrt to a
    Sensor Array for the person
    and based on this information determine the most likely position of the person wrt. to the sensor;
    A good portion of the theory is adapted from the
    Msc Thesis by M. Troost http://alexandria.tue.nl/extra1/afstversl/wsk-i/troostma2013.pdf
    
    The model postulates that an object with a temperature varying
    between t0 and t1 can be identified using Bayesian probability
    
    Where p(o|tø) = p(tø|o)p(o) / p(tø|o)p(o) + p(tø|¬o)p(¬o)
    The normaliser derived from the law of total probability where there are two outcomes
    o being the presence of an object, tø being the temperature sensed. f
    
    M. Troost proposed a digital function for the p(tø|o) basically
    where the probability density function is spread evenly
    over the range of temperatures for the body
    The function for p(tø|¬o) is based on the Probability Density
    function captured by calibrating the model with training data
    It assumes gaussian distribution for the values of the sensors
    which has been observed in experiments for the MLX90620
    The p(o) is again determined based on previous data in this
    class it is assumed to start at 50% probability of the cells
    being unoccupied and adjusted based on actual objects detected during the course of the modelling.
    
    Human skin varies between 25 an 35 degrees this is used as default in sensor calculations
    See http://hypertextbook.com/facts/2001/AbantyFarzana.shtml
    
    Attributes:
    min_body_temp: Min Temperature of body being tracked (defaulted to 25)
    max_body_temp: Max Temperature of body being tracked (defaulted to 35)
    raw_training_data: rawTempArrays [[s1,s2,s3...,s64],[..]]
    """

    def __init__(self, arg_sensor, arg_bin_width=0.5, arg_min_body_temp=25,
                 arg_max_body_temp=35, arg_min_temp=10, arg_max_temp=50, arg_training_data_file=None,
                 arg_use_training_data_input_file=True, arg_training_data=None, arg_prob_o_bar=0.50,
                 arg_object_prob_threshold=0.95,arg_cv_scale =10):

        super(MLX90620MeasurementModel,self).__init__(arg_model_type=MeasurementType.angle,
                                                      arg_subscriptions=arg_sensor,
                                                      arg_name=arg_sensor.sensor_properties.name + '_measurement_model')

        #Training data
        self._input_data_changed = False
        self._raw_training_data = []
        self._training_data_file = ''
        self._use_training_data_file = ''

        self.training_data = []

        # Model Configuration
        self.min_body_temp = arg_min_body_temp
        self.max_body_temp = arg_max_body_temp
        self.training_data_file = arg_training_data_file
        self.raw_training_data = arg_training_data
        self.use_training_data_file = arg_use_training_data_input_file
        self.bin_width = arg_bin_width
        self.min_temp = arg_min_temp
        self.max_temp = arg_max_temp
        #-------- Setting for prob o bar -----------
        self._initial_prob_o_bar = arg_prob_o_bar
        self.reset_prob_o_bar()

        self.sensor_properties = arg_sensor.sensor_properties

        self.object_prob_threshold = arg_object_prob_threshold
        self.last_sensor_reading = {}

        #Initailise Tracking Data
        self.cv_scale = arg_cv_scale
        self._my_cmap = 'binary'
        self._number_of_objects = 0
        self.tracking_circle = None
        self._aoa = np.nan
        self.time_stamp = None
        self.object_tracking_matrix = None
        self.temperature_bins = []
        self.odds_cells_occupied = None
        self.prob_object_present_given_temp = None
        self._prob_prime_t_given_o_bar = []

        self._initialise_temp_counters()
        self.location_measurement = AngularLocationMeasurement(self._aoa, self._number_of_objects, self.time_stamp,
                                                               self.sensor_properties,self.name)

        # Call initialisation functions
        self._create_temperature_bins()
        self.initialise_log_odds()

    def calibrate_measurement_model(self):
        """Method loads training data and creates function to determine p(o|tø) for a temperature ø"""
        self._load_training_data()
        #Update Bins
        self._create_temperature_bins()
        self.initialise_log_odds()

    def reset_prob_o_bar(self):
        self._prob_o_bar_counter = 1
        self.prob_o_bar = self._initial_prob_o_bar

    def initialise_log_odds(self):
        """This function initialises the odds_cells_occupied array it sets the value to 0.0;
        LogOdds of 0.0 --> p(o)/1-p(o) == 1.0 --> p(o) == 0.5, we don't know at the start of the model
        if a cell is occupied or not. """
        temp_log_odds = np.empty(self.sensor_properties.sensing_array_size)
        temp_log_odds.fill(0.0)

        #Load log odds into SensorData class
        self.odds_cells_occupied = OccupancyOdds({TA:0.0, TO:temp_log_odds.tolist()}, self.sensor_properties)

    def create_location_measurement(self, signal):

        self._calculate_log_odds(signal)

        #Create Scaled Version of probabilityData, no interpolation - scaling allows for edge detection to work nicely
        if self.cv_scale > 1:
            self.object_tracking_matrix = ndimage.zoom(self.odds_cells_occupied.prob_cells_occupied_shaped,
                                        self.cv_scale, order=0)
        else:
            self.object_tracking_matrix = self.odds_cells_occupied.prob_cells_occupied_shaped

        #create a colour map to map numbers between 0,1 to a BGR colour map
        mycm = cm.get_cmap(self._my_cmap)

        # Don't need to normalise as already between 0 and 1 and we want
        # to threshold based on our belief that an object is present
        # noinspection PyUnresolvedReferences
        self.object_tracking_matrix = (255 * mycm(self.object_tracking_matrix)).astype('uint8')

        # Convert image to gray scale
        self.object_tracking_matrix = cv2.cvtColor(self.object_tracking_matrix, cv2.COLOR_BGR2GRAY)

        # blur image to highlight contours (small kernel used here)
        self.object_tracking_matrix = cv2.blur(self.object_tracking_matrix, (2, 2))

        # Invert the image and drop all content that is below threshold
        # which is now a % of 255 & invert the image to show
        # Body up as white & background as black.
        ret, self.object_tracking_matrix = cv2.threshold(self.object_tracking_matrix,
                                                       int((1 - self.object_prob_threshold) * 255), 255,
                                                       cv2.THRESH_BINARY_INV)

        # find contours in the image; Only want external ones RETR_EXTERNAL, & just polygon data CHAIN_APPROX_SIMPLE
        contours, hierarchy = cv2.findContours(self.object_tracking_matrix, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #Recast tracking matrix with just outlines of objects
        cv2.drawContours(self.object_tracking_matrix, contours, -1, 128, 1)

        #Compute the bounding circles for returned contours
        bounding_circles = np.array([(cv2.minEnclosingCircle(cnt)) for cnt in contours])

        #look for overlapping circles and merge optimising the size of the new bounding circle
        checked_circles = []
        for bnd_circle in bounding_circles:

            #Add original tracking circles to Tracking Matrix
            cv2.circle(self.object_tracking_matrix, (int(bnd_circle[0][0]), int(bnd_circle[0][1])),
                       int(bnd_circle[1]),64,1)
            cv2.circle(self.object_tracking_matrix, (int(bnd_circle[0][0]), int(bnd_circle[0][1])),
                       int((10 / 40.) * self.cv_scale), 64, -1)

            if len([checked_circle for checked_circle in checked_circles if
                    self._is_overlap(checked_circle, bnd_circle)]) == 0:
                checked_circles.append(bnd_circle)
            else:
                #Take Larger of Two Circles
                for check in [checked_circle for checked_circle in enumerate(checked_circles) if
                              self._is_overlap(checked_circle[1], bnd_circle)]:
                    if bnd_circle[1] > check[1][1]:
                        #bnd_circle[1] = max(g(checked_circle[1],bnd_circle),bnd_circle[1])
                        checked_circles[check[0]] = self._circle_from_2_circles(check[1], bnd_circle)

        #Object Present
        if len(checked_circles) > 0:
            #Simple case - assume only tracking single object we keep only the largest circle
            checked_circles = np.array(checked_circles)
            self.tracking_circle = checked_circles[checked_circles[:, 1].argmax()]

            #Add Tracking Circles to Matrix
            cv2.circle(self.object_tracking_matrix, (int(self.tracking_circle[0][0]), int(self.tracking_circle[0][1])),
                       int(self.tracking_circle[1]), 255, 1)
            cv2.circle(self.object_tracking_matrix, (int(self.tracking_circle[0][0]), int(self.tracking_circle[0][1])),
                       int((10 / 40.) * self.cv_scale), 255, -1)

            # the angle of arrival for a pixel is based on the FOV for the sensor and the number of pixels
            # in the image we can then simply work out the local aoa based on the number of
            # using the center of the tracking circle
            fov = self.sensor_properties.fov
            theta = lambda x: fov/2. - x*(fov / (self.sensor_properties.sensing_array_columns * float(self.cv_scale)))

            self._aoa = theta(self.tracking_circle[0][0])
            self._number_of_objects = 1
        else:

            self._number_of_objects = 0
            self.tracking_circle = None
            self._aoa = np.nan

        # noinspection PyUnresolvedReferences
        self.time_stamp = self.last_sensor_reading.time_stamp
        self.location_measurement = AngularLocationMeasurement(self._aoa,
                                                               self._number_of_objects,
                                                               self.time_stamp,
                                                               self.sensor_properties,self.name)

        #Added Observer Pattern to Class to Allow Fusion Model to Subscribe to Measurement Changes
        self._publish_location_measurement()

        return self.location_measurement

    def visualise_model_data(self):
        self._calculate_occupancy_probability_array()
        temp_training = np.array([i.data_flatten for i in self.training_data]).flatten()

        fig = figure()
        ax = fig.add_subplot(111)
        ax.hist(temp_training ,[i[0] for i in self.temperature_bins],color='lightgreen',normed=True)
        ax2 =ax.twinx()

        ax2.stem([self.sensor_temp_mean,self.mean_ambient_temp ],[1,1],'--.',bottom=0)

        ax2.plot([i[0]+self.bin_width/2. for i in self.temperature_bins],
            [i[1] for i in self.prob_object_present_given_temp],'--o',linewidth=2,color='black')

        ax2.plot([i[0]+self.bin_width/2. for i in self.temperature_bins],
            [stats.norm(self.sensor_temp_mean,self.sensor_temp_std_dev).cdf(i[1])- \
            stats.norm(self.sensor_temp_mean,self.sensor_temp_std_dev).cdf(i[0]) \
            for i in self.temperature_bins],'--o',color='red')
        fig.show()

    @staticmethod
    def _is_overlap(circle1, circle2):
        distance = ((circle1[0][0] - circle2[0][0]) ** 2 + (circle1[0][1] - circle2[0][1]) ** 2) ** 0.5
        return distance < circle1[1] + circle2[1]

    @staticmethod
    def _circle_from_2_circles(circle1, circle2):
        line_midpoint = lambda pt1, pt2: [(pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2]
        line_length = lambda pt1, pt2: np.sqrt(pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2))

        angle = np.arctan2(circle1[0][1] - circle2[0][1], circle1[0][0] - circle2[0][0])

        line_between_extrema = [
            [circle1[0][0] + circle1[1] * np.cos(angle), circle1[0][1] + circle1[1] * np.sin(angle)],
            [circle2[0][0] - circle2[1] * np.cos(angle), circle2[0][1] - circle2[1] * np.sin(angle)]
        ]

        center = line_midpoint(line_between_extrema[0], line_between_extrema[1])

        return (center[0], center[1]), line_length(line_between_extrema[0], line_between_extrema[1]) / 2


    def _update_prob_o_bar(self):
        """Method updates the probability that a cell is unoccupied, based on the complement to the threshold
        for believing that there is an object present e.g. 95% probability is belief --> 5% is threshold for
        believing that there is no object; The prob_o_bar is adjusted as mean based on the count from
        each cell reading"""

        percent_cells_o_bar = len([i for i in self.odds_cells_occupied.prob_cells_occupied_flatten
                                   if i < (1 - self.object_prob_threshold)]) / \
                              float(len(self.odds_cells_occupied.prob_cells_occupied_flatten))
        self._prob_o_bar_counter += 1
        delta = percent_cells_o_bar - self.prob_o_bar
        self.prob_o_bar += delta/float(self._prob_o_bar_counter)

    def _create_temperature_bins(self):
        """Method creates bins of equal size set (bin_width) from the min_temp to max_temp with these boundaries acting as the mid point of the bins
        e.g. 0,2 with bin size 1 -results in-> [(-0.5,0.5),(0.5,1.5),(1.5,2.5)]"""
        self.temperature_bins = [(x - self.bin_width / 2., x + self.bin_width / 2.) for x in
                                np.arange(self.min_temp, self.max_temp + self.bin_width, self.bin_width)]

    def _calculate_log_odds(self, arg_sensor_output):
        """Log odds are calculated based on It = log(P(o|tt)/ 1−P(o|tt)) + log(1-Po/Po) + It-1;
        The function uses the log odds which are frequently used for SLAM mapping approach to determine
        independently for each pixel if it occupied.
        The process is iterative - it requires a prior (Po) and the inverse sensor model P(o|t)
        The prior is initially set to p(¬o) = 0.95 based on experiments
        P(o|t) is determined using the training data.

        The range for log odds is (-∞,+∞) the range for odds is [0,∞) whilst Probability has range [0,1]
        See http://pages.uoregon.edu/aarong/teaching/G4075_Outline/node15.html

        To ensure that objects were being detected and released quickly enough the class
        discards any log odds outside the range [-5,+5]
        --> probabilities of occupation are bound to range [0.0066928509242848563,0.99330714907571516]

        arguments: A sensorData object containing the temperature readings which are required to be analysed.
        """
        self.last_sensor_reading = arg_sensor_output

        for sensor_cell in range(0, len(arg_sensor_output.raw_sensor_data[TO])):
            # noinspection PyBroadException
            try:
                #Determine Probability Sensor is occupied using the calibration data
                prob_sensor_occupied = self._calculate_occupancy_probability(
                     arg_sensor_output.raw_sensor_data[TO][sensor_cell])

            except:
                # If temperature is outside the min/max
                # domain then assume outside min/max body temp domain and default
                # probability to 0
                prob_sensor_occupied = 0.0

            #Stop the log odds popping because of certainties log(0) is -∞
            if prob_sensor_occupied == 0.0:
                prob_sensor_occupied += 1. / np.power(10, 2)
            elif prob_sensor_occupied == 1.0:
                prob_sensor_occupied -= 1. / np.power(10, 2)

            prob_o = 1 - self.prob_o_bar

            self.odds_cells_occupied.raw_sensor_data[TO][sensor_cell] = \
                self.odds_cells_occupied.raw_sensor_data[TO][sensor_cell] + \
                np.log(prob_sensor_occupied) - np.log(1 - prob_sensor_occupied) + np.log(1 - prob_o) - np.log(prob_o)

            #Restrict Log Odds from running to ± Inf
            if self.odds_cells_occupied.raw_sensor_data[TO][sensor_cell] < -5:
                self.odds_cells_occupied.raw_sensor_data[TO][sensor_cell] = -5
            if self.odds_cells_occupied.raw_sensor_data[TO][sensor_cell] > 5:
                self.odds_cells_occupied.raw_sensor_data[TO][sensor_cell] = 5
            self.odds_cells_occupied.time_stamp = arg_sensor_output.time_stamp

        self._update_temp_stats(arg_sensor_output,False)
        self._update_prob_o_bar()

    def _load_training_data(self):
        #Mark the Input Data Changes as False - before attempting to do any actions this
        #ensures that further calls to attributes which require updating will
        #not try and call the failed function again

        self._input_data_changed = False

        if self.use_training_data_file:
            if self.training_data_file is not None:
                with open(self.training_data_file, 'r') as temp_file:
                    #Training Data format {Ta:float, To:[float.....]}
                    self._raw_training_data = [json.loads(i) for i in temp_file.readlines()]
            else:
                raise ValueError('Loading Training Data: No Training Data File Provided')
        else:
            if not self._raw_training_data:
                raise ValueError('Loading Training Data: No Training Data in file ')

        self._initialise_temp_counters()

        #unpack training data
        self.training_data = [MLX90620SensorOutput(i, self.sensor_properties) for i in self._raw_training_data]

        if not self.training_data:
            #Re-set Stats
            self._mean_ambient_temp = np.nan
            self._sensor_temp_std_dev = np.nan
            self._sensor_temp_mean = np.nan
        else:
             for sensor_reading in self.training_data:
                self._update_temp_stats(arg_sensor_reading=sensor_reading)

    def _initialise_temp_counters(self):

        self._mean_ambient_temp = 0.0
        self._sensor_temp_std_dev = 0.0
        self._sensor_temp_mean = 0.0
        #Re-Initialise One Pass Counters
        self._sum_object_temp1 = 0.0
        self._count_object_temp = 0
        self._sum_ambient_temp1 = 0.0
        self._count_ambient_temp =0
        self._M2 = 0 # Variance


    def _update_temp_stats(self,arg_sensor_reading, update_all_cells=True):
        # Updates the temperature statistics to allow the sensor to operate whilst
        # temperatures in environment are changing; This replaces the static calculations
        # proposed by Troost et al for there off line object tracking implementation

        # Derivation of Mean and Standard Deviation from Running Counters - This is naive and can suffer from
        # cancellation and lead to poor precision in result.
        # sum_x2 = sum_x2 + x*x
        # sum_x1 = sum_x1 + x
        # count_x += 1
        # mean = sum_x1 / count_x
        # stdev = sqrt((sum_x2 / count_x) - (mean * mean))
        # http://stackoverflow.com/questions/1174984/how-to-efficiently-calculate-a-running-standard-deviation

        #Ambient Counters
        self._sum_ambient_temp1 += arg_sensor_reading.ambient_data
        self._count_ambient_temp += 1
        self._mean_ambient_temp = self._sum_ambient_temp1 / float(self._count_ambient_temp)

        #Sensor Counters
        if update_all_cells:
            temp_readings = arg_sensor_reading.data_flatten
        else:
            temp_readings = [j for (i,j) in zip(self.odds_cells_occupied.prob_cells_occupied_flatten,
                                                arg_sensor_reading.data_flatten) if i <= 1- self.object_prob_threshold]

        self._sum_object_temp1 += sum(temp_readings)
        temporary_measurement_count = self._count_object_temp
        self._count_object_temp += len(temp_readings)

        if self._count_object_temp == 0:
            pass  # Don't change the stats if no temperature readings to update.
        elif self._count_object_temp == 1:
            self._sensor_temp_mean = self._sum_object_temp1
            self._sensor_temp_std_dev = 0.0
        else:
            # Updated to use the Online algorithm from Wikipedia
            # http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance based on Welford and Kruth
            measurement_counter = temporary_measurement_count
            for temp_reading in temp_readings:
                measurement_counter += 1
                delta = temp_reading - self._sensor_temp_mean
                self._sensor_temp_mean += delta/float(measurement_counter)
                self._M2 += delta*(temp_reading-self._sensor_temp_mean)

            #self._sensor_temp_mean = self._sum_object_temp1 / float(self._count_object_temp)
            #self._sensor_temp_std_dev = \
            #    np.power(self._sum_object_temp2/float(self._count_object_temp) -
            #             np.power(self._sensor_temp_mean,2),1/2.)

            self._sensor_temp_std_dev = np.power(self._M2/float(self._count_object_temp),1/2.)

    def _discrete_temp_given_occupancy(self, temp_bin):
        # Internal method used to calculate the raw probability that a temperature is sensed if an
        # object is present taken from http://alexandria.tue.nl/extra1/afstversl/wsk-i/troostma2013.pdf §6.2
        # the PMF is uniform over the range of temperatures for the object - use this information to derive the
        # probability mass for a discrete temperature bin
        # F(t|O = o) = { t−Tmin Tmax−Tmin for t ∈ [Tmin,Tmax]; 1 for t ≥ Tmax; 0 for t < Tmin }
        # P'(t|o) = F(t+0.125|O = o)−F(t−0.125|O = o)

        def cdf_temp_given_occupancy(arg_calc_temp, arg_calc_min_temp, arg_calc_max_temp):
            # The cumulative density function represented here creates a pdf which
            # is centered around the medium of the min max temperature has a uniform probability of occurring across
            # all the temperatures and and has 0 probability of occurring outside the temperature range
            if arg_calc_temp < arg_calc_min_temp:
                calc_return_value = 0
            elif arg_calc_temp >= arg_calc_max_temp:
                calc_return_value = 1
            else:
                # don't check argCalcMaxTemp - argCalcMinTemp is not 0, because then previous statements always true
                if arg_calc_max_temp - arg_calc_min_temp != 0:
                    calc_return_value = (arg_calc_temp - arg_calc_min_temp) / \
                                float(arg_calc_max_temp - arg_calc_min_temp)
                else:
                    raise ObjectIdentificationException
            return calc_return_value

            #Calculate the probability of Temperature Reading given occupancy of body between min and max temperature
        #by taking the difference of the Cumulative Density probability defined in _calcTemp
        return_value = cdf_temp_given_occupancy(temp_bin[1], self.min_body_temp, self.max_body_temp) - \
            cdf_temp_given_occupancy(temp_bin[0], self.min_body_temp, self.max_body_temp)

        return return_value


    def _calculate_occupancy_probability(self,arg_detected_temperature):
        """Calculate the probability of cell being occupied based on formula suggested by Troost et al
        Calculate p'(t|¬o);
        p'(t|¬o) = F(t+binsize/2|O = ¬o)−F(t−binsize/2|O = ¬o)
        F(t|¬o)= 1/2 ( 1 + erf(t-Tamb/sqrt(2απ**2)))
        """
        try:
            temperature_bin = [i for i in self.temperature_bins
                                if i[0] <= arg_detected_temperature <= i[1]][0]

            #The sensor ambient temp is constantly above the background reading.......
            #need to think about this a bit more // self.mean_ambient_temp replaced with sensorTempMean
            prob_prime_t_given_o_bar = \
                stats.norm(self.sensor_temp_mean, self.sensor_temp_std_dev).cdf(temperature_bin[1])\
                - stats.norm(self.sensor_temp_mean, self.sensor_temp_std_dev).cdf(temperature_bin[0])

            po_t_num = self._discrete_temp_given_occupancy(temperature_bin) * (1 - self.prob_o_bar)

            po_t_denom = self._discrete_temp_given_occupancy(temperature_bin) * (1 - self.prob_o_bar) +\
                         prob_prime_t_given_o_bar * self.prob_o_bar

            if po_t_denom == 0:
                #Prevents Errors when denominator is 0 -> Happens when tø is outside of domain predicted by training data
                #and outside object temp domain; In this scenario assume p(o|tø) = 0
                prob_sensor_occupied =  0.0
            else:
                prob_sensor_occupied = po_t_num / po_t_denom

        except TypeError:
            # Temperature outside of expected range default to 0.0
            prob_sensor_occupied = 0.0

        return prob_sensor_occupied


    def _calculate_occupancy_probability_array(self):
        """Function creates an array of temperature bins, zipped with the the probability
        that there is an object in the bin"""
        po_t = []
        for temp_bin in enumerate(self.temperature_bins):
            po_t.append([temp_bin[1],self._calculate_occupancy_probability(temp_bin[1][0]+self.bin_width/2.)])
        self.prob_object_present_given_temp = po_t



    def __str__(self):
        #Don't print the arrays in String otherwise can't read any
        return str(
            [(k, v if np.iterable(v) == 0 else '[..]' if len(v) > 0 else '[]') for k, v in self.__dict__.items()])


    # Decorate functions to ensure if training data
    # changes the new data is loaded into the model

    @property
    def mean_ambient_temp(self):
        if self._input_data_changed:
            self._load_training_data()
        return self._mean_ambient_temp

    @property
    def sensor_temp_std_dev(self):
        if self._input_data_changed:
            self._load_training_data()
        return self._sensor_temp_std_dev

    @property
    def sensor_temp_mean(self):
        if self._input_data_changed:
            self._load_training_data()
        return self._sensor_temp_mean

    @property
    def training_data_file(self):
        return self._training_data_file

    @training_data_file.setter
    def training_data_file(self, arg_file):
        self._training_data_file = arg_file
        self._input_data_changed = True

    @property
    def raw_training_data(self):
        return self._raw_training_data

    @raw_training_data.setter
    def raw_training_data(self, arg_array):
        self._raw_training_data = arg_array
        self._input_data_changed = True

    @property
    def use_training_data_file(self):
        return self._use_training_data_file

    @use_training_data_file.setter
    def use_training_data_file(self, arg_bool):
        self._use_training_data_file = arg_bool
        self._input_data_changed = True
