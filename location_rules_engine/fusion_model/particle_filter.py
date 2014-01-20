#!/usr/bin/env python

"""
Module Docstring
"""
from location_rules_engine.measurement_models.location_measurement import AngularLocationMeasurement

__author__ = 'jim'
__version__ = '0.0.1'

import random
import time
import logging

from numpy import pi, array, dot, inf
from shapely.geometry import Polygon, Point, LineString

from globals.position import Position
from globals.utility_functions import gaussian, \
    cartesian_to_polar,polar_to_cartesian, distance_between_points
from globals.global_constants import *


class Particle(object):
    """Particles represents guesses about the location of an object,
    the class contains set-up, movement and measurement methods to
    facilitate the guessing of particles fitness"""

    def __init__(self, arg_forward_speed, arg_world_dimensions=(1, 1),
                 arg_world_usable_area = Polygon([[0,0],[0,1],[1,1],[1,0]]),
                 arg_is_robot=False):
        # Set particle default noise parameters
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

        # Set up belief parameters
        self.location_measurements = []
        self.w = 1

        # Store world dimensions for particle & supporting polygons
        self.world_dimensions = arg_world_dimensions
        self.world_usable_area = arg_world_usable_area


        # Allow particles to be measured irrespective of their position;
        # Allows simulation robots to be created from class
        self.is_robot = arg_is_robot

        # Set speed randomly with an average approximating the supplied argument
        self.forward_speed = random.uniform(0,2*arg_forward_speed)

        #Place Particle randomly in the world with a random speed
        self.location = Position(0, 0, 0)
        self.set_random_location()

    def set_random_location(self):
        self.location = Position(random.uniform(0, self.world_dimensions[0]),
                                 random.uniform(0, self.world_dimensions[1]),
                                 random.uniform(0, 2 * pi))

    def set_noise(self, new_t_noise, new_f_noise, new_s_noise):
        # Set up noise parameters
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)

    @property
    def noise_tfs(self):
        return self.turn_noise, self.forward_noise, self.sense_noise


    def set_location(self, arg_x, arg_y, arg_orientation):
        # Set particle position directly
        if arg_x < 0 or arg_x > self.world_dimensions[0]:
            raise ValueError('X coordinate out of bound')
        if arg_y < 0 or arg_y > self.world_dimensions[1]:
            raise ValueError('Y coordinate out of bound')
        if arg_orientation < 0 or arg_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.location = Position(float(arg_x), float(arg_y), float(arg_orientation))

    def sense(self, arg_sensors):
        #Creates location measurements for each of the provided sensors
        self.location_measurements = {}
        for sensor_properties in arg_sensors:
            if sensor_properties.location.xy == self.location.xy:
                # location not well defined if particle @ same point as sensor
                # set local_aoa to be the rotation of the sensor
                local_aoa = -sensor_properties.location.heading
            else:
                local_aoa = self._sense_local(sensor_properties)[1]

            number_of_objects = 1

            if self.is_robot:
                local_aoa += random.gauss(0.0, self.sense_noise)
                #Particle falls outside sensor active zone then if its not a robot there are no measurements results
                if not (sensor_properties.measurement_boundary_poly.contains(Point(self.location.xy)) or
                        sensor_properties.measurement_boundary_poly.intersects(Point(self.location.xy))):
                    local_aoa = -inf
                    number_of_objects = 0

            self.location_measurements[sensor_properties.name] = \
                AngularLocationMeasurement(local_aoa, number_of_objects, time.time(), sensor_properties,'pir_filter')
        return self.location_measurements

    def _sense_local(self, arg_sensor_properties):
        # move sensor to origin and rotate to 0 degrees heading to calculate local aoa
        global_particle_location = array([self.location.x, self.location.y])

        local_particle_location = (dot(arg_sensor_properties.rotation_matrix,
                                       global_particle_location + arg_sensor_properties.translation_vector))[0]

        return_value = cartesian_to_polar(local_particle_location[0],
                                          local_particle_location[1], response_units=AngleUnits.radians)
        return return_value

    def move(self, arg_turn, arg_forward_speed):

        if self.is_robot:
            robot_world_iterations_max = 5
        else:
            robot_world_iterations_max = 1

        # function to check if particle still in world_map after movement
        #move_in_map = lambda x, y: 0 < x < self.world_dimensions[0] and 0 < y < self.world_dimensions[1]

        move_in_map = lambda x, y: self.world_usable_area.contains(Point(x,y))

        #turn particle with noise
        self.location.heading = self.location.heading + float(arg_turn) + random.gauss(0.0, self.turn_noise)
        self.location.heading %= 2 * pi

        #calculate the distance the particle will travel
        dist = float(arg_forward_speed) + random.gauss(0.0, self.forward_noise)
        dx,dy = polar_to_cartesian(dist,self.location.heading)

        spinning_counter = 0


        #keep robots in the world truncate distance on current heading to the usable area boundary
        move_keeps_particle_in_map = False


        while not move_keeps_particle_in_map and spinning_counter <= robot_world_iterations_max:

            if not move_in_map(self.location.x + dx, self.location.y + dy):
                # make sure there is some noise and spin the particle randomly to try and make it remain in the world
                self.location.heading = self.location.heading + random.gauss(pi, self.turn_noise)
                self.location.heading %= 2 * pi
                dx, dy = polar_to_cartesian(self.forward_speed, self.location.heading)
                movement_vector = LineString([[self.location.x, self.location.y], [self.location.x+dx, self.location.y+dy]])
                dist_within_usable_area = round(movement_vector.intersection(self.world_usable_area).length,2)
                #if dist_within_usable_area > 0:
                self.forward_speed = cmp(dist,0)*dist_within_usable_area #Deal with -ve speeds

            else:
                move_keeps_particle_in_map = True

            spinning_counter += 1


        if spinning_counter <= robot_world_iterations_max or not self.is_robot:
            self.location.x += dx
            self.location.y += dy
        else:
            logging.warning('Warning - robot not moved')


    def calculate_weight(self, arg_location_measurements):
        """The importance sampling for the the particle is based on the weight calculated here"""

        # Each particle is assumed to be true and we calculate the probability of getting the the measurement (Z) for
        # based on the sensor noise
        # Watch student dave and his bayesian ninjas http://www.youtube.com/watch?v=O-lAJVra1PU

        # This function is usually taken to be gaussian but can be problematic if the scenario is highly no linear
        # see http://proquest.safaribooksonline.com/book/math/9781118287804/part-iii-monte-carlo-methods/
        self.w = 1.0

        if self.world_usable_area.contains(Point([self.location.x,self.location.y])):
             for measurement in arg_location_measurements.values():
                if measurement.global_position != -inf:
                    self.w *= gaussian(measurement.global_position,
                                                       self.sense_noise,
                                                       self.location_measurements[
                                                           measurement.sensor_properties.name].global_position)
                    if not Point(self.location.x,self.location.y).within(self.location_measurements[
                                    measurement.sensor_properties.name].sensor_properties.measurement_boundary_poly):
                        self.w /=pow(10,100)



                else:
                    # If we don't have any information about the
                    # object from a particular sensor we don't adjust weight
                    if Point(self.location.x,self.location.y).within(self.location_measurements[
                                                           measurement.sensor_properties.name].sensor_properties.measurement_boundary_poly):
                        self.w /=pow(10,100)

        else:
            self.w = 0

class ParticleFilterModelParameters(object):
    """Holds model parameters for Particle Filter"""
    def __init__(self,arg_number_of_particles=250,
                      arg_sensor_noise=1.0,
                      arg_turn_noise=0.05,
                      arg_forward_noise=0.05,
                      arg_speed = 0.1,
                      arg_turn = 0.01,
                      arg_mean_confidence_threshold=0.95,
                      arg_world_dimensions=(1,1),
                      arg_world_usable_area=[[]]):

        self.number_of_particles = arg_number_of_particles

        self.sensor_noise = arg_sensor_noise
        self.turn_noise = arg_turn_noise
        self.forward_noise = arg_forward_noise

        self.forward_speed = arg_speed
        self.iteration_turn = arg_turn

        self.mean_confidence_threshold = arg_mean_confidence_threshold

        self._world_usable_area = None

        self.world_dimensions = arg_world_dimensions
        self.world_usable_area = arg_world_usable_area


    @property
    def world_usable_area(self):
        return self._world_usable_area

    @world_usable_area.setter
    def world_usable_area(self, arg_world_usable_area):
        self._world_usable_area = arg_world_usable_area
        try:
            self._world_usable_area = Polygon(self.world_extremities).intersection(
                Polygon(self._world_usable_area))

        except ValueError:
            self._world_usable_area = Polygon(self.world_extremities)

    @property
    def world_extremities(self):
        return [[0, 0],
                [self.world_dimensions[0], 0],
                list(self.world_dimensions), [0, self.world_dimensions[1]]]



class ParticleFilter(object):
    """Particle Filter Model holds methods to set up and run particle filter in simulation and
    online mode"""

    def __init__(self,arg_model_parameters, arg_sensors):
        self.model_parameters = arg_model_parameters
        self.particles = []
        self.sensors = arg_sensors
        self.create_particles()

    def create_particles(self):
        """Creates fresh particle set"""
        self.particles=[]
        for n in range(0,self.model_parameters.number_of_particles):
            self.particles.append(self._create_particle())

    def _create_particle(self,arg_is_robot=False):
        """Particle creation function to wrap up creation of new particles"""
        particle = Particle(arg_forward_speed=self.model_parameters.forward_speed,
                            arg_world_dimensions=self.model_parameters.world_dimensions,
                            arg_world_usable_area=self.model_parameters.world_usable_area,
                            arg_is_robot=arg_is_robot)

        particle.set_noise(new_t_noise=self.model_parameters.turn_noise,
                           new_f_noise=self.model_parameters.forward_noise,
                           new_s_noise=self.model_parameters.sensor_noise)
        return particle

    def compute_location_mean(self):
        """Helper function to support calculation of mean position and confidence
        adapted from https://github.com/mjl/particle_filter_demo"""

        m_x, m_y, m_count = 0, 0, 0
        for p in self.particles:
            m_count += p.w
            m_x += p.location.x * p.w
            m_y += p.location.y * p.w

        if m_count == 0:
            return_value = (-1, -1, 0,False)
        else:
            m_x /= m_count
            m_y /= m_count

            # Now compute how good that mean is -- check how many particles
            # actually are in the immediate vicinity
            m_count = 0

            for p in self.particles:
                 if distance_between_points((p.location.x, p.location.y), (m_x, m_y)) < 1:
                    m_count += 1

            return_value = (m_x,
                            m_y,
                            m_count/float(self.model_parameters.number_of_particles),
                            m_count >
                            self.model_parameters.number_of_particles *
                            self.model_parameters.mean_confidence_threshold)

        return return_value

    def move_particles(self):
        """Method to move particles and create location measurements for their new positions"""
        for particle in self.particles:
            particle_turn = random.gauss(self.model_parameters.iteration_turn,particle.turn_noise)
            #particle_forward = random.gauss(particle.forward_speed,particle.forward_noise)
            particle.move(particle_turn,particle.forward_speed)
            particle.sense(self.sensors)

    def resample_particles(self,arg_location_measurements):
        """Method re-samples the particles based on their importance based on the location_measurements received from the
        physical sensors (and or simulated sensors in the case of a robot"""

        for particle in self.particles:
            particle.calculate_weight(arg_location_measurements)

        if len([x for x in self.particles if x.w ==0]) / float(len(self.particles)) > 0.5:
            new_particles = []

            for particle in self.particles:
                if particle.w > 0:
                    new_particles.append(particle)
                else:
                    new_particle = self._create_particle()
                    new_particle.sense(self.sensors)
                    new_particle.calculate_weight(arg_location_measurements)
                    new_particles.append(new_particle)

            self.particles = new_particles

        new_particles = []
        total_weight = sum([particle.w for particle in self.particles])
        max_weight = max([particle.w for particle in self.particles])

        if total_weight > 0:
            # Resample particles with probability of sampling based on weight
            # Method used is Wheel adapted from tutorials in  Udacity AI Course https://www.udacity.com/course/cs373
            index = int(random.random() * len(self.particles))
            beta = 0.0

            for i in range(self.model_parameters.number_of_particles): # Resampling implements P(X|Z) <- P(Z/X)P(X)
                beta += random.random() * 2.0 * max_weight
                while beta > self.particles[index].w:
                    beta -= self.particles[index].w
                    index = (index + 1) % len(self.particles)

                new_particle = self._create_particle()
                new_particle.w = self.particles[index].w
                new_particle.set_location(self.particles[index].location.x,
                                        self.particles[index].location.y,
                                        self.particles[index].location.heading)

                new_particles.append(new_particle)

            self.particles = new_particles
        else:
            # No fit particles create fresh sample
            self.create_particles()

