# 16-12-13 Jim Brown
# Small Utility function to dump sensor data to local file to facilitate testing

import time
import logging

import serial


clicked = False
tempResults = []

try:
    print 'starting sensor'
    logging.debug('Starting serial')
    ser = serial.Serial('/dev/tty.Node4-DevB',115200,timeout=0.1)
    tempfile = open('/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/location_sensors/utilities/rawData-'+ str(time.time()) + '.json', 'w')

    timeOut = time.time() +  10
    print 'processing lines'
    while time.time() < timeOut:
        try:
            
            line = ser.readline()
            tempfile.write(line)
            
            #time.sleep(0.01)
        except serial.SerialTimeoutException:
            print('Data could not be read')
            #time.sleep(1)
        except:
            print 'another error'
    
    print 'closing'
except KeyboardInterrupt:
    print 'Terminating serial early'

finally:
    ser.close()
    tempfile.close()

