from test.observer_class import TestMeasurementObserver

__author__ = 'jim'

import time
import json
import globals.global_constants as gc
import logging
import globals.geoJson_utility_functions as guf
from globals.building_model import Level
import location_rules_engine.measurement_models.mlx90620_measurement_model as oim
from pydispatch import dispatcher
from location_rules_engine.fusion_model.angular_measurement_fusion_model import \
    AngularFusionModelParams, AngularMeasurementFusionModel
import cv2
import threading
import sys


def process_sensor_readings(stop_event, my_sensor):
    logging.info('%s - sensor: Process Started on %s', my_sensor.sensor_properties.name, my_sensor.port)
    my_sensor.open()
    file_name = gc.logs + my_sensor.sensor_properties.name + '-' + str(time.time()) + '.json'
    with open(file_name, 'w') as f:
        while not stop_event.is_set():
            try:
                sensor_reading = my_sensor.make_observation()
                logging.debug('%s - sensor: reading taken @ %s', my_sensor.sensor_properties.name,
                              sensor_reading.time_stamp)

                json.dump(obj={'time_stamp': sensor_reading.time_stamp,
                               'raw_readings': sensor_reading.raw_sensor_data}, fp=f)
                f.write('\n')

            except IOError as e:
                logging.warning('%s: I/O error(%s): %s', my_sensor.sensor_properties.name,
                                e.errno, e.strerror)
            except:
                logging.warning('%s: Measurement failed: %s, %s', my_sensor.sensor_properties.name,
                                str(sys.exc_info()[0]), sys.exc_info()[1])
            finally:
                stop_event.wait(0.1)


def run_ips_for_room(time_to_run, pres=100,
                     num_particles=100,
                     sensor_noise=0.05,
                     turn_noise=0.05,
                     forward_noise=0.05,
                     me_speed=0.1,
                     me_turn=0.01,
                     floor_plan=gc.config_floor_plan_file,
                     sensor_config_file=gc.sensor_config_file,
                     sensor_image_scale=50,
                     room_name='kitchen',
                     sensor_creation_mode=gc.SensorCreationMode.emulation,
                     include_fusion_model=True, include_sensor_vis=False):
    """Start up test emulator and use it to pass data to the Sensor class and then onwards to the measurement model"""
    logging.info('Starting IPS for: %s, %s', room_name, time.ctime())

    #-=-=-=-=-=-=- Sensors -=-=-=-=-=-=-=-=-=
    logging.info('Creating sensors from %s', sensor_config_file)
    sensor_objects = guf.create_mlx90620_sensors(guf.load_map(sensor_config_file), sensor_creation_mode)

    #Remove Sensors for Other Rooms
    for sen_obj in [k for k, x in sensor_objects.items()
                    if x['sensor_properties'].location.frame_of_reference_name != room_name]:
        del sensor_objects[sen_obj]

    logging.info('Extracting %s container from %s', room_name, floor_plan)
    floor_plan_geojson_file = floor_plan
    floor_plan = Level(guf.load_map(floor_plan_geojson_file))


    #-=-=-=-=-=-=- Angular Measurement Models -=-=-=-=-=-=-=-=-=
    logging.info('Creating Angular Location Measurement Models')

    measurement_models = []
    sensor_observers = []
    for sensor in sensor_objects.keys():
        node_oim = oim.MLX90620MeasurementModel(sensor_objects[sensor]['sensor'])

        node_oim.training_data_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/' + \
                                      'location_rules_engine/measurement_models/training_data/training_data.json'
        node_oim.cv_scale = sensor_image_scale
        node_oim.calibrate_measurement_model()

        measurement_models.append(node_oim)

        #Set up Observer To log oim data -=-=-==-=-=-=-=-=-=-=-=-==
        file_h = open(gc.logs + node_oim.name + '-' + str(time.time()) + '.json', 'w')
        sensor_observers.append(TestMeasurementObserver(node_oim, sensor, 'log_angular_location_measurements', file_h))

        if include_sensor_vis:
            sensor_observers.append(TestMeasurementObserver(node_oim, sensor, 'test_readings'))


    #logging.debug('Connections %s', dispatcher.connections)


    #-=-=-=-=-=-=- Fusion Models -=-=-=-=-=-=-=-=-=
    if include_fusion_model:
        logging.info('Creating Fusions Location Measurement Model for %s', room_name)

        fusion_model_params = AngularFusionModelParams(arg_number_of_particles=num_particles,
                                                       arg_sensor_noise=sensor_noise,
                                                       arg_turn_noise=turn_noise,
                                                       arg_forward_noise=forward_noise,
                                                       arg_speed=me_speed,
                                                       arg_turn=me_turn,
                                                       arg_mean_confidence_threshold=0.95,
                                                       arg_discard_old_measurements=10,
                                                       arg_cvscale=pres)

        fusion_model = AngularMeasurementFusionModel(arg_name='Fusion Model ' + room_name,
                                                     arg_related_measurement_models=measurement_models,
                                                     arg_container=floor_plan.rooms['kitchen'],
                                                     arg_model_params=fusion_model_params)

        #Set up logging
        file_h = open(gc.logs + fusion_model.name + '-' + str(time.time()) + '.json', 'w')
        sensor_observers.append(TestMeasurementObserver(fusion_model, fusion_model.name,
                                                        'log_pos_location_measurements', file_h))



    logging.debug('\nConnections %s\n\n', dispatcher.connections)

    #Threading used to allow consuming tasks to be completed in the background.
    stop_event = threading.Event()
    stop_event.clear()

    #-=-=-=-=-=-=- Initialise Connections -=-=-=-=-=-=-=-=-=
    sensor_reading_threads = []
    for sensor in sensor_objects:
        #If we have emulator objects start these wait


        if sensor_objects[sensor]['emulator']:
            sensor_objects[sensor]['emulator'].start_sensor()

        sensor_thread = threading.Thread(target=process_sensor_readings,
                                         args=(stop_event, sensor_objects[sensor]['sensor'],))
        sensor_thread.daemon = True
        sensor_thread.start()

        sensor_reading_threads.append(sensor_thread)

    #Ensure the process terminates with the function

    timeOut = time.time() + time_to_run

    loop_counter = 0

    try:
        while time.time() < timeOut:
            loop_counter += 1
            if loop_counter % 10000 == 0:
                logging.debug('Processing loop: %s, time reaming %s', loop_counter, timeOut - time.time())
            if include_fusion_model:
                fusion_model.visualise_measurements()
                cv2.waitKey(1)
    except KeyboardInterrupt:
        logging.info('Force shutdown...')

    logging.info('closing and tidying up...')

    logging.info('Killing Sensor Threads')
    stop_event.set()
    time.sleep(1)

    while sum([my_thread.is_alive() for my_thread in sensor_reading_threads]) > 0:
        logging.warning('Waiting for Threads to Terminate - %s',
                        [my_thread.is_alive() for my_thread in sensor_reading_threads])
        time.sleep(1)
    for obs in sensor_observers:
        if obs.file_handle:
            obs.file_handle.close()

    try:
        return_value = 0
    except:
        logging.error("Unexpected error: %s - %s", str(sys.exc_info()[0]), str(sys.exc_info()[1]))
        return_value = 1
    finally:
        for sensor_key in sensor_objects.keys():
            for fn in (
                        "sensor_objects['" + sensor_key + "']['sensor'].close()",
                        "sensor_objects['" + sensor_key + "']['emulator'].stop_sensor()",
                        "sensor_objects['" + sensor_key + "']['emulator'].close_port()"):
                try:
                    exec (fn)
                except:
                    logging.warning("%s clean up error: %s, %s", sensor_key, sys.exc_info()[0], sys.exc_info()[1])

                    continue

        try:
            cv2.destroyAllWindows()
        except:
            logging.warning("clean up error: %s, %s", sys.exc_info()[0], sys.exc_info()[1])

    return return_value


if __name__ == '__main__':
    log_format = '%(asctime)s - %(levelname)s - %(filename)s - %(message)s'
    logging.basicConfig(level=logging.INFO, format=log_format) #, filename=gc.logs + 'run_ips.log')
    run_ips_for_room(60, num_particles=100,sensor_config_file=gc.sensor_config_file_dir + 'two_sensor_config.json',
                     sensor_creation_mode=gc.SensorCreationMode.emulation,include_sensor_vis=True,
                     include_fusion_model=True,pres=50)
