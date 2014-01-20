#!/usr/bin/env python
"""
Module holds different types of Location Measurements current supporting
Positional and Angular
"""

from numpy import inf
from numpy.core.umath import isinf
from globals.utility_functions import modulo_heading
import globals.global_constants as gc

__author__ = 'jim'
__version__ = '0.0.1'


class LocationMeasurement(object):
    """Base Class for Location Measurements"""

    def __init__(self, arg_location_measurement_type, arg_measurement_units, arg_source_model):
        self.location_measurement_type = arg_location_measurement_type
        self.measurement_units = arg_measurement_units
        self.source_model = arg_source_model


class PositionalLocationMeasurement(LocationMeasurement):
    def __init__(self, arg_position,
                 arg_time_stamp,
                 arg_source_model,
                 arg_percent_within_area=0,
                 arg_number_of_objects=0):
        super(PositionalLocationMeasurement, self).__init__(gc.MeasurementType.coords,
                                                            gc.DistanceUnits.meters, arg_source_model)
        self.position = arg_position
        self.time_stamp = arg_time_stamp
        self.number_of_objects = arg_number_of_objects
        self.percent_within_area =  arg_percent_within_area



class AngularLocationMeasurement(LocationMeasurement):
    """Infrared location measurement, aoa is provided in radians"""

    def __init__(self, arg_local_aoa, arg_number_of_objects, arg_time_stamp, arg_sensor_properties, arg_source_model):
        super(AngularLocationMeasurement, self).__init__(gc.MeasurementType.angle, gc.AngleUnits.radians,
                                                         arg_source_model)
        self.time_stamp = arg_time_stamp
        self.sensor_properties = arg_sensor_properties
        self._aoa = 0
        #self.global_aoa = 0
        self.local_position = arg_local_aoa
        self.number_of_objects = arg_number_of_objects

    @property
    def global_position(self):
        #don't need to move origin to get rotation as this translation is implicit in the other measurements
        #change so only executed once when the local aoa is set.
        if isinf(self.local_position):
            return_value = -inf
        else:


            # This linear algebra is nice but actually all that we need to do is add the delta between the global reading
            # and the local reading to get the correct result should be less expensive in terms of calculation as well

            return_value = modulo_heading(self.local_position + self.sensor_properties.location.heading)

            #local_sensor_position = array([sqrt(2) * cos(self.local_aoa), sqrt(2) * sin(self.local_aoa)])
            #global_particle_position = dot(linalg.inv(self.sensor_properties.rotation_matrix), local_sensor_position)
            #return_value = cartesian_to_polar(global_particle_position[0][0],global_particle_position[0][1],
            #                     response_units=AngleUnits.radians)[1]

        return return_value


    @property
    def local_position(self):
        return self._aoa

    @local_position.setter
    def local_position(self, arg_aoa):
        if isinf(arg_aoa):
            self._aoa = -inf
        else:
            self._aoa = modulo_heading(arg_aoa)
