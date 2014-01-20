#!/usr/bin/env python

"""
Module contains functions to load and process geoJson

"""
__author__ = 'jim'
__version__ = '0.0.1'

import json
import sys
import logging

from globals.position import Position
from location_sensors.sensor_properties import MLX90620SensorProperties
from location_sensors.sensor import MLX90620
import location_sensors.sensor_emulator.thermal_sensor_emulator as tse
from globals.global_constants import *



#----------------------------------------Load GeoJson ---------------------------------------------------------------

def load_map(file_name):
    """load map for specified room"""
    map_data = {}
    try:
        with open(file_name,'r') as fs:
            map_data = json.load(fs)
    except IOError as e:
        logging.error("I/O error({0}): {1}".format(e.errno, e.strerror))
    except:
        logging.error("Unexpected error: %s, %s" ,str(sys.exc_info()[0]) ,str(sys.exc_info()[1]))
    return map_data


def create_mlx90620_sensors(arg_geojson_sensor_config, arg_creation_mode):
    return_sensors = {}

    for sensors_config in [feature for feature in arg_geojson_sensor_config['features']
                           if feature['properties']['geomType'] == 'sensor']:

        node_position = Position(arg_x=sensors_config['geometry']['coordinates'][0],
                                 arg_y=sensors_config['geometry']['coordinates'][1],
                                 arg_heading=sensors_config['properties']['positionProperties']['heading'],
                                 arg_reference_frame=PositionReferenceFrame.room_plan if
                                 sensors_config['properties']['positionProperties'][
                                     'positionReferenceFrame'] == 'room' else
                                 PositionReferenceFrame.level_plan,
                                 arg_dataum=Datum.NA,
                                 arg_reference_frame_name=sensors_config['properties']['positionProperties'][
                                     'referenceFrameName'])

        node_config = MLX90620SensorProperties(arg_name=sensors_config['properties']['name'],
                                               arg_location=node_position,
                                               arg_port=None,
                                               arg_baud_rate=sensors_config['properties']['connectionProperties'][
                                                   'baud'],
                                               arg_time_out=sensors_config['properties']['connectionProperties'][
                                                   'timeOut'],
                                               arg_sensor_flush=sensors_config[
                                                   'properties']['connectionProperties']['flush'],
                                               arg_sensing_array_size=sensors_config[
                                                   'properties']['sensorProperties']['arrayCells'],
                                               arg_sensing_array_columns=sensors_config[
                                                   'properties']['sensorProperties']['arrayCols'],
                                               arg_sensing_array_rows=sensors_config[
                                                   'properties']['sensorProperties']['arrayRows'],
                                               arg_sensing_array_transpose=sensors_config[
                                                   'properties']['sensorProperties']['arrayTranspose'],
                                               arg_fov=sensors_config['properties']['sensorProperties']['fov'],
                                               arg_cv_scale=sensors_config['properties']['sensorProperties']['cvScale'],
                                               arg_range=sensors_config['properties']['sensorProperties']['range'])
        node = MLX90620(node_config)
        return_sensors[node_config.name] = {'sensor_properties': node_config, 'sensor': node}

        if arg_creation_mode == SensorCreationMode.emulation:
            test_emulator = tse.VirtualMLX90620Hw()
            test_emulator.load_sensor_readings(sensors_config['properties']['connectionProperties']['emulationFile'])
            node_config.port = test_emulator.create_port()
            return_sensors[node_config.name]['emulator'] = test_emulator

        else:
            node_config.port = sensors_config['properties']['connectionProperties']['port']
            return_sensors[node_config.name]['emulator'] = None

        node.port = node_config.port

    return return_sensors