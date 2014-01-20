from pydispatch import dispatcher

__author__ = 'jim'

import logging
import cv2
import sys
import json


class TestMeasurementObserver(object):
    def __init__(self, arg_oim, arg_obs_name, arg_function='test_readings',arg_file_handle=None):
        if arg_function == 'test_readings':
            dispatcher.connect(self.test_readings, dispatcher.Any, arg_oim)
        elif arg_function == 'log_angular_location_measurements':
            dispatcher.connect(self.log_angular_location_measurements, dispatcher.Any, arg_oim)
        elif arg_function == 'log_pos_location_measurements':
            dispatcher.connect(self.log_pos_location_measurements, dispatcher.Any, arg_oim)
        else:
            logging.warning('argument %s is can not be set to be triggered by pydispatcher')
        self.file_handle = arg_file_handle
        self.name = arg_obs_name

    def test_readings(self, sender):
        logging.debug('%s: Trapped Signal:', self.name)
        cv2.imshow('tracking window' + self.name, sender.object_tracking_matrix)
        cv2.imshow('prob image' + self.name, sender.odds_cells_occupied.data_image_array)
        cv2.waitKey(1)

    def log_angular_location_measurements(self, sender, signal):
        if self.file_handle:
            try:
                dump_obj= {'time_stamp':signal.time_stamp,
                       'source_model':signal.source_model,
                       'number_of_object':signal.number_of_objects,
                       'local_position':signal.local_position,
                       'global_position':signal.global_position,
                       'location_measurement_type':signal.location_measurement_type,
                       'measurement_units':signal.measurement_units}

                json.dump(obj=dump_obj,fp=self.file_handle)
                self.file_handle.write('\n')

            except:
                logging.error("Unexpected error: %s - %s", str(sys.exc_info()[0]), str(sys.exc_info()[1]))

            #print signal.__dict__
            #json.dump(obj=signal.__dict__,fp=self.file_handle)
            #self.file_handle.write('\n')
        else:
            logging.warning('No File Handle provided, logging to central log: %s - %s',sender.name, signal)

    def log_pos_location_measurements(self, sender, signal):
        if self.file_handle:
            try:
                dump_obj= {'time_stamp':signal.time_stamp,
                       'source_model':signal.source_model,
                       'position': {'x': signal.position.x,'y': signal.position.y},
                       'location_measurement_type':signal.location_measurement_type,
                       'measurement_units':signal.measurement_units}

                #'number_of_object':signal.number_of_objects,

                json.dump(obj=dump_obj,fp=self.file_handle)
                self.file_handle.write('\n')
                #print signal.__dict__

            except:
                logging.error("Unexpected error: %s - %s", str(sys.exc_info()[0]), str(sys.exc_info()[1]))

            #print signal.__dict__
            #json.dump(obj=signal.__dict__,fp=self.file_handle)
            #self.file_handle.write('\n')
        else:
            logging.warning('No File Handle provided, logging to central log: %s - %s',sender.name, signal)