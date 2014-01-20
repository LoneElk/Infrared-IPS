__author__ = 'jim'

import unittest
import copy
import sys
import random
import time
import logging

from numpy import pi, sqrt, isinf, zeros, array, rint, dot, linalg,log
from shapely.geometry import Polygon
import shapely.affinity as sa
import cv2

import location_rules_engine.fusion_model.particle_filter as pf
from location_sensors.sensor_properties import InfraredSensorProperties
from location_sensors.sensor import InfraredSensor
from globals.global_constants import *
from globals.position import Position
from globals.building_model import Level
from globals.geoJson_utility_functions  import load_map




# -------------------------------------------------------- Kalman Filter ------------------------------------------
def prediction(x, F, u, P):
        # prediction
        x = dot(F,x) + u # ******** State Prediction **********
        P = dot(dot(F,P), F.transpose()) # ********** Covariance Predication: Predict How Much Error *********
        return x,P

def measurement(arg_measurement, H, x, P, R, I):
        # measurement update
        Z = array([arg_measurement]) # ********** Measurement Vector ********
        y = Z.transpose() - dot(H , x) # ********** Innovation:  compare prediction with reality *********
        S = dot(dot(H,P),H.transpose()) + R # ********** Innovation Covariance: Compare Real Error against Prediction
        K = dot(dot(P,H.transpose()), linalg.inv(S)) # *********** Kalman Gain: Moderate the Prediction ************
        x = x + dot(K , y) # **************** State Update (new estimate) **************
        P = dot((I - dot(K , H)) ,P) # ********************** Covarience Update ************************
        return x,P

#------------------------------------------------------------- Draw Function --------------------------------------

def draw(scale,sensors, robbie_the_robot, particles, pres, particle_location_mean,frame_rate,usable_area ):
    cnt = lambda x: rint(array(x)*pres).astype(int)

    img = zeros((scale[1]*pres,scale[0]*pres,3)).astype('uint8')
    img +=255

    im_copies = []
    if usable_area != [[]]:
        cv2.polylines(img,[cnt(usable_area)],isClosed=True, color=(0,0,0), thickness=2)

    for i, sensor in enumerate(sensors):
        img_copy = copy.deepcopy(img)
        cv2.fillPoly(img_copy,cnt(sensor.measurement_boundary),(128,128,128))
        im_copies.append(img_copy)

    max_w = max([p.w for p in particles])

    img_copy = copy.deepcopy(img)
    for particle in particles:
        col = 255.*log(particle.w+1)/log(max_w+1)

        cv2.circle(img_copy,(int(particle.location.x*pres),int(particle.location.y*pres)),5,(0,col,0),-1)

    im_copies.append(img_copy)

    for i,overlay in enumerate(im_copies):
        opacity = 0.4
        cv2.addWeighted(overlay, opacity, img, 1-opacity, 0, img)




    for sensor in sensors:
        cv2.circle(img,(int(sensor.location.x*pres),int(sensor.location.y*pres)),10,(0,0,255),-1)
        cv2.line(img,\
            (int(sensor.location.x*pres),int(sensor.location.y*pres)),\
            (int(robbie_the_robot.location.x*pres),int(robbie_the_robot.location.y*pres)),\
            (0,0,64),1)

    cv2.circle(img,(int(robbie_the_robot.location.x*pres),int(robbie_the_robot.location.y*pres)),min(10,int(pres/10.)),(0,255,0),1)

    m_x, m_y, percent, good = particle_location_mean

    if m_x != -1 and m_y != -1:
        if good == True:
            cv2.circle(img,(int(m_x*pres),int(m_y*pres)),min(12,int(pres/8.)),(255,255,0),1)
        else:
            cv2.circle(img,(int(m_x*pres),int(m_y*pres)),min(12,int(pres/8.)),(64,64,0),1)

    cv2.imshow('test',cv2.flip(img,0))
    cv2.waitKey(1)
    time.sleep(1/float(frame_rate))


def run_pir_simulation(arg_num_iter = 1000, pres =100,
        num_particles =100,
        sensor_noise = 0.05,
        turn_noise =0.05,
        forward_noise =0.05,
        sensor_range = 5,
        sensor_fov = pi/3.,
        frame_rate =1000.,
        me_speed = 0.1,
        me_turn =0.01,
        floor_plan =config_floor_plan_file):
    """Simulation of the Particle Filter"""
    logging.info("Starting Particle Filter simulation")

    try:
        floor_plan_geojson_file = floor_plan
        floor_plan = Level(load_map(floor_plan_geojson_file))

        x1,y1,x2,y2 = floor_plan.rooms['kitchen'].walls.polygon.bounds

        world_size = (x2/float(floor_plan.scale),y2/float(floor_plan.scale))

        usable_area = sa.scale(floor_plan.rooms['kitchen'].usable_area.polygon,
                                                       1/float(floor_plan.scale),1/float(floor_plan.scale),
                                                       origin=(0,0,0)).exterior.coords[:]
        usable_area_bounds =sa.scale(floor_plan.rooms['kitchen'].usable_area.polygon,
                                                       1/float(floor_plan.scale),1/float(floor_plan.scale),
                                                       origin=(0,0,0)).bounds

        pir_model_params = pf.ParticleFilterModelParameters(arg_number_of_particles=num_particles,
                                              arg_world_dimensions= world_size,
                                              arg_sensor_noise=sensor_noise,
                                              arg_turn_noise=turn_noise,
                                              arg_forward_noise=forward_noise,
                                              arg_speed=me_speed, arg_turn=me_turn,
                                              arg_world_usable_area=usable_area)


        sensors = [InfraredSensorProperties('Sensor1', Position(usable_area_bounds[0]+0.25, usable_area_bounds[1]+1, pi/4.),
                                                           MeasurementType.temperature, sensor_fov, sensor_range, 16),
                   InfraredSensorProperties('Sensor2', Position(usable_area_bounds[2]-0.25, usable_area_bounds[1]+0.25, 3 * pi /4.),
                                                           MeasurementType.temperature,sensor_fov, sensor_range, 16) ,
                   InfraredSensorProperties('Sensor3', Position(usable_area_bounds[2]-1.25, usable_area_bounds[3]-0.25, -3 * pi /4.),
                                                           MeasurementType.temperature,sensor_fov, sensor_range, 16),
                   InfraredSensorProperties('Sensor4', Position(usable_area_bounds[0]+0.25, usable_area_bounds[3]-0.25, -pi*1/4.),
                                                    MeasurementType.temperature, pi / 3., sensor_range, 16)]

        for my_sensor in sensors:
            my_sensor.measurement_boundary_poly = sa.scale(floor_plan.rooms['kitchen'].usable_area.polygon,
                                                       1/float(floor_plan.scale),1/float(floor_plan.scale),
                                                       origin=(0,0,0)).intersection(my_sensor.measurement_boundary_poly)
            my_sensor.measurement_boundary = [my_sensor.measurement_boundary_poly.exterior.coords[:]]


        pir_model = pf.ParticleFilter(pir_model_params,sensors)

        robbie_the_robot = pf.Particle(me_speed, arg_world_dimensions=world_size,
                                       arg_world_usable_area=pir_model_params.world_usable_area,
                                       arg_is_robot=True)

        robbie_the_robot.set_noise(0.1,0.0,sensor_noise)
        robbie_the_robot.set_location(2.5,2.5,0)

        draw(world_size,sensors, robbie_the_robot, pir_model.particles, pres, pir_model.compute_location_mean(),
             frame_rate,usable_area)

        results =[]

        for t in range(0,arg_num_iter):
            robbie_the_robot.move(random.gauss(me_turn,turn_noise),me_speed)
            robbie_the_robot.sense(sensors)

            pir_model.move_particles()
            draw(world_size,sensors, robbie_the_robot, pir_model.particles, pres, pir_model.compute_location_mean(),
             frame_rate,usable_area)
            pir_model.resample_particles(robbie_the_robot.location_measurements)

            draw(world_size,sensors, robbie_the_robot, pir_model.particles, pres, pir_model.compute_location_mean(),
             frame_rate,usable_area)
            results.append({'robbie':robbie_the_robot.location.xyh,'mean':pir_model.compute_location_mean()})

            return_value = 0

    except:
       return_value = 1
       logging.error("Unexpected error:" + str(sys.exc_info()[0]) + str(sys.exc_info()[1]))
       raise  sys.exc_info()[0](sys.exc_info()[1])

    logging.info("Ending PIR simulation & cleaning up")
    cv2.destroyAllWindows()

    return return_value, results


class ParticleTests(unittest.TestCase):

    def setUp(self):
        self.particle = pf.Particle(arg_forward_speed=0)

    def test_setup(self):
        self.assertEquals(self.particle.forward_speed,0)
        self.assertEquals(self.particle.noise_tfs,(0,0,0))
        self.assertTrue(0 < self.particle.location.x < 1 and 0 < self.particle.location.y < 1)
        self.assertEquals(self.particle.w,1)

    def test_set_random_position(self):
        temp_position = self.particle.location.xyh
        self.particle.set_random_location()
        self.assertNotEqual(temp_position,self.particle.location.xyh)
        self.assertTrue(0 < self.particle.location.x < 1 and 0 < self.particle.location.y < 1)

    def test_noise(self):
        self.particle.set_noise(1.5,2.6,3.7)
        self.assertEquals(self.particle.noise_tfs,(1.5,2.6,3.7))

    def test_set_position(self):
        temp_position = self.particle.location.xyh
        pos = (0.9,0.08,3.007)
        self.particle.set_location(*pos)
        self.assertNotEqual(temp_position,self.particle.location.xyh)
        self.assertEquals(pos,self.particle.location.xyh)
        self.assertTrue(0 < self.particle.location.x < 1 and 0 < self.particle.location.y < 1)

    def test_sense(self):
        self.sensors = [InfraredSensorProperties('Sensor1', Position(0, 0, 0), MeasurementType.temperature, pi / 3., 1, 16),
                        InfraredSensorProperties('Sensor2', Position(1, 1, 3 * pi / 2), MeasurementType.temperature,
                                                     pi / 3., 1, 16)]

        self.particle.set_location(0,0,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,0)

        self.particle.set_location(0,1,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,pi/2.)

        self.particle.set_location(1,1,0)
        self.assertAlmostEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,pi/4.,5)

        self.particle.set_location(1,0,0)
        self.assertAlmostEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,0,5)


        #Translation
        self.sensors[0].location.x = 1
        self.sensors[0].location.y = 1
        self.sensors[0].calculate_measurement_boundary()

        self.particle.set_location(0,0,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,-pi*3/4.)

        self.particle.set_location(0,1,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,pi)


        self.particle.set_location(1,1,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,0)

        self.particle.set_location(1,0,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,-pi/2.)

        # Rotation
        self.particle.set_location(0,0,pi/2.)
        self.sensors[0].location.heading = pi/2
        self.sensors[0].location.x = 0
        self.sensors[0].location.y = 0
        self.sensors[0].calculate_measurement_boundary()

        self.particle.set_location(0,0,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,0)

        self.particle.set_location(0,1,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,pi/2.)

        self.particle.set_location(1,1,0)
        self.assertAlmostEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,pi/4.,5)

        self.particle.set_location(1,0,0)
        self.assertAlmostEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,0,5)

        #Translation & Rotation + Second Sensor
        self.sensors[0].location.x = 1
        self.sensors[0].location.y = 1
        self.particle.set_location(0,0,3*pi/2.)
        self.sensors[0].calculate_measurement_boundary()

        self.particle.set_location(0,0,0)
        self.assertAlmostEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,pi*5/4.,6)
        self.assertAlmostEquals(self.particle.sense(self.sensors)['Sensor2'].global_position,pi*5/4.,6)

        self.particle.set_location(0,1,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,pi)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor2'].global_position,pi)

        self.particle.set_location(1,1,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor2'].global_position,0)

        self.particle.set_location(1,0,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,-pi/2.)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor2'].global_position,pi*3/2.)

        #Sensor Boundary
        self.sensors[0].location.x = 0
        self.sensors[0].location.y = 0
        self.particle.sense_noise = 0
        self.particle.is_robot = True

        self.sensors[0].calculate_measurement_boundary()
        self.sensors[1].calculate_measurement_boundary()


        self.particle.set_location(0,0,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,0)


        self.particle.set_location(0,0.99,0)
        self.assertEquals(self.particle.sense(self.sensors)['Sensor1'].global_position,pi/2.)

        self.particle.set_location(1,1,0)
        self.assertTrue(isinf(self.particle.sense(self.sensors)['Sensor1'].global_position))

        self.particle.set_location(1,0,0)
        self.assertTrue(isinf(self.particle.sense(self.sensors)['Sensor1'].global_position))

        # Noise
        self.particle.sense_noise = 10
        self.particle.set_location(0,0,0)
        self.assertTrue(0<abs(self.particle.sense(self.sensors)['Sensor1'].global_position)<self.particle.sense_noise)

    def test_move(self):

        #Check movement without noise
        self.particle.set_location(0,0,0)
        self.particle.set_noise(0,0,0)
        self.particle.is_robot = False

        self.particle.move(0,0.5)
        self.particle.move(0,0.5)
        self.particle.move(pi/2.,1)
        self.particle.move(pi/2.,1)
        self.particle.move(pi/2.,1)
        self.assertItemsEqual(self.particle.location.xy,(0,0))
        self.particle.move(pi*3/4.,sqrt(0.5))
        self.assertItemsEqual(self.particle.location.xy,(0.5,0.5))

        #Check movement for robot with noise
        self.particle.set_location(0.5,0.5,0)
        self.particle.set_noise(1,1,0.01)
        self.particle.is_robot=True

        for i in range(0,10):
            self.particle.move(0,0.01)

        self.assertTrue(0<=self.particle.location.x<=1 and 0<=self.particle.location.y<=1)

        #Check movement for particle with noise
        self.particle.set_location(0,0,0)
        self.particle.is_robot=False
        for i in range(0,10):
            self.particle.move(0,0.5)

    def test_weight(self):

        #Two sensors
        self.sensors = [InfraredSensorProperties('Sensor1', Position(0, 0, pi/4.), MeasurementType.temperature, pi / 3., 1, 16),
            InfraredSensorProperties('Sensor2', Position(1, 1, -pi*3/4.), MeasurementType.temperature, pi / 3., 1, 16)]

        self.particle.set_location(0.5,0.5,3*pi/2.)
        self.particle.is_robot = False
        measurements = self.particle.sense(self.sensors)
        # add tests for measurements

class TestParticleModel(unittest.TestCase):

    def setUp(self):
        self.pir_model_params = pf.ParticleFilterModelParameters(arg_world_usable_area=[[0.1,0.1],[0.1,0.9],[0.9,0.9],[0.9,0.1]])

        self.sensors = [InfraredSensor(InfraredSensorProperties('Sensor1',
                                                           Position(0, 0, pi/4.),
                                                           MeasurementType.temperature,pi/3.,1,16))]

        self.pir_model = pf.ParticleFilter(self.pir_model_params,self.sensors)

    def test_setup(self):


        test_polygon = [[0.25, 0.25], [0.75, 0.25], [0.75, 0.75], [0.25, 0.75]]
        self.pir_model_params.world_usable_area = test_polygon
        self.assertTrue(self.pir_model_params.world_usable_area.equals(Polygon(test_polygon)))

        test_polygon = [[0.9648914887204741, 0.25854189518631476], [0.0, 0.0],
                                           [0.2585418951863151, 0.9648914887204741],
                                           [0.29028467725445867, 0.9569403357322099],
                                           [0.3826834323650859, 0.9238795325112884],
                                           [0.47139673682599365, 0.8819212643483572],
                                           [0.5555702330195982, 0.831469612302548],
                                           [0.6343932841636415, 0.7730104533627402],
                                           [0.7071067811865436, 0.7071067811865515],
                                           [0.7730104533627332, 0.63439328416365],
                                           [0.8314696123025418, 0.5555702330196074],
                                           [0.8819212643483519, 0.4713967368260034],
                                           [0.9238795325112841, 0.3826834323650961],
                                           [0.9569403357322068, 0.29028467725446927],
                                           [0.9648914887204741, 0.25854189518631476]]

        self.pir_model_params.world_usable_area = test_polygon
        self.assertTrue(self.pir_model_params.world_usable_area.equals(Polygon(test_polygon)))

        test_polygon = [[-1,-1], [-1, 2], [2, 2], [2, -1]]
        self.pir_model_params.world_usable_area = test_polygon
        self.assertTrue(
            self.pir_model_params.world_usable_area.equals(Polygon(self.pir_model_params.world_extremities)))




        with self.assertRaises(TypeError):
            self.pir_model_params.world_usable_area = 55


class TestSimulation(unittest.TestCase):

    def test_pir_simulation(self):
        self.assertEquals(0,run_pir_simulation(5)[0])


if __name__ == '__main__':
    unittest.main()
