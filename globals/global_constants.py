__author__ = 'jim'

from enum import Enum

config_floor_plan_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/configuration/floorplan1.json'
sensor_config_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/configuration/sensor_config.json'
single_sensor_config_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/configuration/single_sensor_config.json'
sensor_config_file_dir = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/configuration/'
logs = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/logs/'



TA = 'Ta'
TO = 'To'
wall_const = 'wall'
floor_const = 'floor'
wall_thickness_const = 0.25

# Sensor classifications taken from  http://www.dsmforum.org/events/dsm13/Papers/AlMamun.pdf
class SensorType(Enum):
    infrared = 0

class MeasurementType(Enum):
    temperature = 0
    distance = 1
    angle = 2
    coords = 3

class DistanceUnits(Enum):
    pixel = 0
    centimeters = 1
    meters = 2

class AngleUnits(Enum):
    radians = 0
    degrees = 1

class TemperatureUnits(Enum):
    celsius = 0


#Reference frames taken from  http://www.geos.ed.ac.uk/~gisteac/eeo_agi/2011-12/3_worboys_21102011.pdf BIM Taxonomy
class PositionReferenceFrame(Enum):
    longitude_latitude = 0
    building = 1
    level_plan = 2
    room_plan = 3
    easting_northing = 4
    na = 5

class Datum(Enum):
    NA = 0
    WGS84 = 1


class ParticleFilterMode(Enum):
    simulation = 0
    online = 1

class ContainerType(Enum):
    room = 0
    level = 1

class SurfaceType(Enum):
    wall = 0
    floor = 1
    abstract = 2

class GeomType(Enum):
    surface = 0
    barrier = 1

class SensorCreationMode(Enum):
    standard = 0
    emulation = 1
