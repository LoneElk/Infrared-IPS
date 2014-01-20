#!/usr/bin/env python

"""
Module holds classes related to various sensor properties & configuration used across the various layers of the IPS
Application
see http://www.w3.org/2005/Incubator/ssn/XGR-ssn-20110628/ for Sensor Ontology
"""
__author__ = 'jim'
__version__ = '0.0.1'

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
import time
from cv2 import cvtColor, COLOR_BGR2RGB
import logging

from numpy import array, min, arange,linspace, max, pi, dot, tan, linalg
from pylab import cm
from scipy import interpolate
from shapely.geometry import Point, Polygon
from globals.position import Position
from globals.utility_functions import calculate_transformation_matrices


# ----------------- Constants -----------------
from globals.global_constants import *


# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
# Sensor Output Data

class SensorOutput(object):
    def __init__(self,arg_raw_sensor_data, arg_sensor_data_units, arg_measurement_type):
        self.time_stamp = time.time()
        self.raw_sensor_data = arg_raw_sensor_data
        self.sensor_data_units = arg_sensor_data_units
        self.measurement_type = arg_measurement_type


class MLX90620SensorOutput(SensorOutput):
    """Class used to define structure of sensor data"""

    def __init__(self,arg_raw_sensor_data,
                 arg_sensor_properties, arg_cv_scale=10, arg_cmap='hot'):

        super(MLX90620SensorOutput,self).__init__(arg_raw_sensor_data,
                                                MeasurementType.temperature,TemperatureUnits.celsius)

        try:
            if sorted([TA,TO]) != sorted(arg_raw_sensor_data.keys()):
                raise ValueError('Sensor data keys not:' + TO + ' ' + TA)
            if len(arg_raw_sensor_data[TO]) != arg_sensor_properties.sensing_array_size:
                raise ValueError('SensorDataArray length does not match SensorConfig Expected Size')

            # MLX90620 Sensor Properties are used to format display of raw data
            self.sensor_properties = arg_sensor_properties

            # Constants to support display of data using cv2
            self.cv_scale = arg_cv_scale
            self.cmap = arg_cmap
            self.set_display_constants()

        except:
            logging.error('Sensor Data is not in the correct format dict:{ "Ta":float, "To":[float,...]}.')
            raise

    def _shape_sensor_data(self,arg_flat_sensor_data):
        if self.sensor_properties.sensing_array_transpose:
            return_value = array(arg_flat_sensor_data).reshape(self.sensor_properties.sensing_array_columns,
                                                                  self.sensor_properties.sensing_array_rows).transpose()
        else:
            return_value = array(arg_flat_sensor_data).reshape(self.sensor_properties.sensing_array_rows,
                                                                  self.sensor_properties.sensing_array_columns)
        return return_value
    @property
    def ambient_data(self):
        """Displays the ambient temperature data"""
        return self.raw_sensor_data[TA]

    @property
    def data_shaped(self):
        """Extracts the Sensor data from the raw input and displays them as
        nxm array described in the SensorConfig settings"""
        return self._shape_sensor_data(self.data_flatten)
    @property
    def data_flatten(self):
        """Extracts the Sensor data from the raw input and displays them
        as nxm array described in the SensorConfig settings"""
        temp = array(self.raw_sensor_data[TO])
        return temp

    @property
    def data_image_array(self):
        """Helper function to structure the data into an interpolated CV2 array"""
        new_kernel = interpolate.RectBivariateSpline(self._rows,self._cols,self.data_shaped)
        kernel_out = new_kernel(self._yy_rows,self._xx_cols )
        my_cm = cm.get_cmap(self.cmap)
        normed_kernel = (kernel_out - min(kernel_out)) / (max(kernel_out) - min(kernel_out))
        mapped_kernel = (255 * my_cm(normed_kernel)).astype('uint8')
        return cvtColor(mapped_kernel,COLOR_BGR2RGB)

    def set_display_constants(self):
        #Interpolation Constants
        self._cols = arange(self.sensor_properties.sensing_array_columns)
        self._rows = arange(self.sensor_properties.sensing_array_rows)
        self._xx_cols = linspace(self._cols.min(),self._cols.max(),
                                 self.sensor_properties.sensing_array_columns*self.sensor_properties.cv_scale)
        self._yy_rows = linspace(self._rows.min(),self._rows.max(),
                                 self.sensor_properties.sensing_array_rows*self.sensor_properties.cv_scale)

    def __str__(self):
        return str(self.__dict__)

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=
# Sensor Properties
class SensorProperties(object):
    def __init__(self,arg_sensor_name,arg_sensor_type,arg_sensor_location):
        self.name = arg_sensor_name
        self.type = arg_sensor_type
        self.location = arg_sensor_location

class InfraredSensorProperties(SensorProperties):
    def __init__(self,arg_name, arg_location, arg_measurement_units,
                 arg_fov, arg_range, arg_sensing_array_size):
        super(InfraredSensorProperties,self).__init__(arg_name,SensorType.infrared,arg_location)

        self.location = arg_location

        self.measurement_type = MeasurementType.temperature
        self.measurement_units = arg_measurement_units

        #Measurement Properties
        self.fov = arg_fov
        self.range = arg_range
        self.number_of_cells = arg_sensing_array_size

        #Precalulate Geometric properties of sensors to optimise particle filter
        self.measurement_boundary_poly = None
        self.measurement_boundary = []
        self.rotation_matrix, self.translation_vector = calculate_transformation_matrices(self.location)
        self.calculate_measurement_boundary()

    def calculate_measurement_boundary(self):
        self.rotation_matrix, self.translation_vector = calculate_transformation_matrices(self.location)
        opp_length = tan(self.fov/2.)*self.range
        triangle = [[0, 0], [self.range, -opp_length],
                    [self.range, opp_length]]

        triangle = [(dot(linalg.inv(self.rotation_matrix),array(x)) -
                            self.translation_vector).tolist()[0] for x in triangle]


        shapely_triangle = Polygon(triangle)
        shapely_circle = Point(self.location.x, self.location.y).buffer(self.range)
        try:
            #Grow the polygon by small amount as original results were missing key points due to rounding
            self.measurement_boundary_poly = shapely_triangle.intersection(shapely_circle)
            self.measurement_boundary = [[list(x) for x in
                                         self.measurement_boundary_poly.exterior.coords[:]]]

        except:
            self.measurement_boundary = [[]]
            self.measurement_boundary_poly = None

        return self.measurement_boundary




class MLX90620SensorProperties(InfraredSensorProperties):
    """Holds the configuration information for a MLX90620 sensor"""

    def __init__(self, arg_name=None, arg_location=Position(0, 0, 0), arg_port=None,
                 arg_baud_rate=115200, arg_time_out=0.1, arg_sensor_flush=False, arg_sensing_array_size=64,
                 arg_time_between_ambient_temp=1, arg_sensing_array_columns=16, arg_sensing_array_rows=4,
                 arg_fov=pi / 3., arg_sensing_array_transpose=True, arg_cv_scale=50, arg_range=8):

        super(MLX90620SensorProperties,self).__init__(arg_name, arg_location, TemperatureUnits.celsius,
                                                      arg_fov,arg_range, arg_sensing_array_size)

        if arg_sensing_array_size != arg_sensing_array_columns * arg_sensing_array_rows:
            raise ValueError("""SensorDataArray length does not match Row,
            Column sizing for the sensor or ambient temperature not provided""")

        # Sensor Data Connection Properties
        self.time_out = arg_time_out
        self.flush = arg_sensor_flush
        self.port=arg_port
        self.baud_rate=arg_baud_rate
        self.time_out=arg_time_out

        # Sensor Physical Properties
        self.sensing_array_size = arg_sensing_array_size
        self.sensing_array_columns = arg_sensing_array_columns
        self.sensing_array_rows = arg_sensing_array_rows
        self.sensing_array_transpose=arg_sensing_array_transpose
        self.time_between_ambient_temp = arg_time_between_ambient_temp

        # Sensor Data Visualisation
        self.cv_scale = arg_cv_scale


    def __str__(self):
        return str(self.__dict__)

