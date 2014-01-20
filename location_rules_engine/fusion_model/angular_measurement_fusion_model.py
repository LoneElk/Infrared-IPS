#!/usr/bin/env python

"""
Fusion Measurement Model, integrates two or more angular readings and returns a Positional Location Measurement
"""
__author__ = 'jim'
__version__ = '0.0.1'

import copy
import time
import logging
import numpy as np
import cv2
import shapely.affinity as sa

from location_rules_engine.measurement_models.measurement_model import MeasurementModel
from location_rules_engine.measurement_models.location_measurement import \
    AngularLocationMeasurement, PositionalLocationMeasurement
from location_rules_engine.fusion_model.particle_filter import ParticleFilter, ParticleFilterModelParameters
from globals.position import Position
import globals.global_constants as gc


class AngularFusionModelParams(object):
    """Holds model parameters for Angular Fusion Model"""

    def __init__(self, arg_number_of_particles=250,
                 arg_sensor_noise=1.0,
                 arg_turn_noise=0.05,
                 arg_forward_noise=0.05,
                 arg_speed=0.1,
                 arg_turn=0.01,
                 arg_mean_confidence_threshold=0.95,
                 arg_discard_old_measurements = 10,
                 arg_cvscale=100):
        self.number_of_particles = arg_number_of_particles

        self.sensor_noise = arg_sensor_noise
        self.turn_noise = arg_turn_noise
        self.forward_noise = arg_forward_noise

        self.forward_speed = arg_speed
        self.iteration_turn = arg_turn

        self.mean_confidence_threshold = \
            arg_mean_confidence_threshold

        self.discard_old_measurements = arg_discard_old_measurements
        self.cvscale = arg_cvscale

class AngularMeasurementFusionModel(MeasurementModel):
    """This model is responsible for co-ordinating readings from the various
    sensors to create a positional location measurement
    """

    def __init__(self, arg_name,
                 arg_related_measurement_models,
                 arg_container,
                 arg_model_params):

        #Subscribe to Location Events from related Measurement Models
        super(AngularMeasurementFusionModel, self).__init__(arg_name,
                                                            gc.MeasurementType.coords,
                                                            arg_related_measurement_models)

        #Set up Container could be Floor or Room
        self.container = arg_container

        # Set up Model Senors Properties and Initial Angular Measurements
        self.sensor_properties = {}
        self.angular_location_measurements = {}
        if isinstance(arg_related_measurement_models, list):
            self.subscription_up_data_frequency = len(arg_related_measurement_models)
            for rel_mm in arg_related_measurement_models:
                self.subscribe_to_measurement_data_source(rel_mm)
        else:
            self.subscribe_to_measurement_data_source(arg_related_measurement_models)
            self.subscription_up_data_frequency = 1

        #Set up model parameters
        self.particle_filter_model_param = None
        self.particle_filter_sensor_properties = None
        self.pir_model = None
        self.calibrate_measurement_model(arg_model_params)
        self.fusion_model_params = arg_model_params
        self.m_x =0
        self.m_y = 0
        self.percent = 0
        self.good = 0
        self.current_data_visualised = False
        self.update_counter = 0



    def calibrate_measurement_model(self, arg_model_params):
        x1, y1, x2, y2 = self.container.floor.polygon.bounds
        world_size = (x2 / float(self.container.scale), y2 / float(self.container.scale))
        usable_area_poly = sa.scale(self.container.usable_area.polygon,
                                     1 / float(self.container.scale), 1 / float(self.container.scale),
                                     origin=(0, 0, 0))

        self.particle_filter_model_param = ParticleFilterModelParameters(arg_number_of_particles=arg_model_params.number_of_particles,
                                                         arg_world_dimensions=world_size,
                                                         arg_sensor_noise=arg_model_params.sensor_noise,
                                                         arg_turn_noise=arg_model_params.turn_noise,
                                                         arg_forward_noise=arg_model_params.forward_noise,
                                                         arg_speed=arg_model_params.forward_speed,
                                                         arg_turn=arg_model_params.iteration_turn,
                                                         arg_world_usable_area=usable_area_poly.exterior.coords[:])

        # Sensor Geometry needs to be translated to position relative to origin taken to be top left corner of room
        # on floor plan i.e. bounding box for usable area

        min_from_origin = min([np.sqrt(pow(j[0],2)+pow(j[1],2)) for j in usable_area_poly.exterior.coords[:]])
        minx, miny = [i for i in usable_area_poly.exterior.coords[:]
                      if np.sqrt(pow(i[0],2)+pow(i[1],2)) ==min_from_origin ][0]

        # self.sensor_properties = copy.deepcopy(self.sensor_properties)
        # 
        for k, sensor_prop in self.sensor_properties.items():
            self.sensor_properties[k].measurement_boundary_poly = usable_area_poly.intersection(
                self.sensor_properties[k].measurement_boundary_poly)
            self.sensor_properties[k].measurement_boundary = [
                self.sensor_properties[k].measurement_boundary_poly.exterior.coords[:]]

        self.pir_model = ParticleFilter(self.particle_filter_model_param, self.sensor_properties.values())

    def create_location_measurement(self, signal):



        logging.debug('Create Location Measurement Triggered')

        self.update_counter += 1

        if self.update_counter % self.subscription_up_data_frequency == 0:
            logging.debug('Model not updated')
            if isinstance(signal, AngularLocationMeasurement):
                self.update_angular_location_measurements(signal)

                self.pir_model.move_particles()
                self.pir_model.resample_particles(self.angular_location_measurements)
            else:
                logging.warning('Location Measurement not in correct format')

            self.m_x, self.m_y, self.percent, self.good  = self.pir_model.compute_location_mean()

            if self.container.container_type == gc.ContainerType.level:
                ref_frame = gc.PositionReferenceFrame.level_plan
            else:
                ref_frame = gc.PositionReferenceFrame.room_plan

            self.location_measurement = PositionalLocationMeasurement(
                arg_position=Position(self.m_x, self.m_y, 0, ref_frame, gc.Datum.NA, self.container.name),
                arg_time_stamp=time.time(),
                arg_source_model=self.name,
                arg_number_of_objects=1 if self.good else 0,
                arg_percent_within_area=self.percent)
            self.current_data_visualised = False
            self._publish_location_measurement()

            return self.location_measurement
        else:
            logging.debug('Model not updated, current updates level is: %s of %s',
                          self.update_counter % self.subscription_up_data_frequency,
                          self.subscription_up_data_frequency)

            return self.location_measurement

    def subscribe_to_measurement_data_source(self, arg_related_measurement_model):
        self.subscribe_to_measurement_events(arg_related_measurement_model)
        self.sensor_properties[arg_related_measurement_model.name] = \
            arg_related_measurement_model.sensor_properties
        self.angular_location_measurements[arg_related_measurement_model.name] = \
            AngularLocationMeasurement(-np.inf, 0, time.time(),
                                       arg_related_measurement_model.sensor_properties,self.name)

    def disconnect_measurement_data_source(self, arg_related_measurement_model):
        self.disconnect_from_measurement_events(arg_related_measurement_model)
        self.sensor_properties.pop(arg_related_measurement_model.name)
        self.angular_location_measurements.pop(arg_related_measurement_model.name)

    def update_angular_location_measurements(self,signal):
            self.angular_location_measurements[signal.sensor_properties.name] = signal
            # might have to think about voiding old measurement signals
            for k in self.angular_location_measurements.keys():
                if self.angular_location_measurements[k].time_stamp < \
                                time.time() - self.fusion_model_params.discard_old_measurements:
                   #If measurement is older than set threshold bin it and replace with no reading
                   self.angular_location_measurements[k] = \
                       AngularLocationMeasurement(-np.inf,0,time.time(),
                                                  self.angular_location_measurements[k].sensor_properties,self.name)

    def visualise_measurements(self):

        if not self.current_data_visualised:
            self.current_data_visualised = True
            cnt = lambda x: np.rint(np.array(x)*self.fusion_model_params.cvscale).astype(int)

            img = np.zeros((self.particle_filter_model_param.world_dimensions[1]*self.fusion_model_params.cvscale,
                            self.particle_filter_model_param.world_dimensions[0]*self.fusion_model_params.cvscale,3)).astype('uint8')
            img +=255


            im_copies = []
            #Draw Usable Area
            if self.particle_filter_model_param.world_usable_area != [[]]:
                cv2.polylines(img,[cnt(self.particle_filter_model_param.world_usable_area.exterior.coords[:])],
                              isClosed=True, color=(0,0,0), thickness=2)
            #Add sensors
            for k, sensor_prop in self.sensor_properties.items():
                img_copy = copy.deepcopy(img)
                cv2.fillPoly(img_copy,
                             [cnt(self.sensor_properties[k].measurement_boundary_poly.exterior.coords[:])],
                             (128,128,128))
                im_copies.append(img_copy)

            max_w = np.min(np.max([p.w for p in self.pir_model.particles]),0)
            img_copy = copy.deepcopy(img)

            for particle in self.pir_model.particles:

                col = 255.*np.log((particle.w+1)/(max_w+1))
                cv2.circle(img_copy,
                           (int(particle.location.x*self.fusion_model_params.cvscale),
                            int(particle.location.y*self.fusion_model_params.cvscale)),5,(0,col,0),-1)

            im_copies.append(img_copy)

            for i,overlay in enumerate(im_copies):
                opacity = 0.4
                cv2.addWeighted(overlay, opacity, img, 1-opacity, 0, img)

            for sensor in self.sensor_properties.values():
                cv2.circle(img,(int(sensor.location.x*self.fusion_model_params.cvscale),
                                int(sensor.location.y*self.fusion_model_params.cvscale)),10,(0,0,255),-1)



            if(self.m_x != -1 and self.m_y != -1) and not (self.m_x is np.inf or self.m_y is np.inf):
                if self.good:
                    cv2.circle(img,(int(self.m_x*self.fusion_model_params.cvscale),
                                    int(self.m_y*self.fusion_model_params.cvscale)),
                               min(12,int(self.fusion_model_params.cvscale/8.)),(255,255,0),1)
                else:
                    cv2.circle(img,(int(self.m_x*self.fusion_model_params.cvscale),
                                    int(self.m_y*self.fusion_model_params.cvscale)),
                               min(12,int(self.fusion_model_params.cvscale/8.)),(64,64,0),1)

            cv2.imshow('test',cv2.flip(img,0))
            cv2.waitKey(1)
        else:
            logging.debug('Data unchanged visualisation not updated.')
