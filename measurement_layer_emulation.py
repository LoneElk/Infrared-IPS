#!/usr/bin/env python

"""
Script to execute the measurement layer emulation tests
Uses data captures from a sensor and the MLX90620 virtual hardaware
to test the Measurement and Sensor layers of the architecture
"""
__author__ = 'jim'
__version__ = '0.0.1'


from test.test_emulator_sensor import run_test_emulator

#Starts up the test emulator and measurment model
run_test_emulator(60)



