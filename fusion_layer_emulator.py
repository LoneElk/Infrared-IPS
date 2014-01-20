#!/usr/bin/env python

"""
Small script to execute the particle test emulator
No live data to use with this layer yet so the particle filter uses Robbie the Robot (a green blob)
to emulate the user being tracked by the lower layers
The example room is the Kitchen which can be found in the /configuration/floorplan1.json file
"""
__author__ = 'jim'
__version__ = '0.0.1'

from test.test_particle_filter import run_pir_simulation
run_pir_simulation(arg_num_iter=100,sensor_noise=0.1,pres=100,num_particles=750)
