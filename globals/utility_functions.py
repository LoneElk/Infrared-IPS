#!/usr/bin/env python

"""
Module holds utility functions which are used across the Infrared-IPS
"""
__author__ = 'jim'
__version__ = '0.0.1'

import sys
import logging

from numpy import arctan2, hypot, degrees, pi, array, sin, cos, exp, sqrt, radians
import shapely.geometry.multipolygon
from shapely.geometry import Polygon, MultiPolygon

from global_constants import *



def cartesian_to_polar(x, y, response_units=AngleUnits.degrees):
    """Convert from rectangular (x,y) to polar (r,w),
    r = sqrt(x^2 + y^2), w = arctan(y/x) = [-\pi,\pi] = [-180,180]
    Returns (w,r) r in radians unless deg set to 1
    http://www.mathsisfun.com/polar-cartesian-coordinates.html"""

    angle = arctan2(y, x)

    if response_units == AngleUnits.degrees:
        return [hypot(x, y), degrees(angle)] # radian if deg=0; degree if deg=1
    else:
        return [hypot(x, y), angle]


def polar_to_cartesian(arg_dist, arg_heading, arg_heading_units=AngleUnits.radians):
    if arg_heading_units == AngleUnits.degrees:
        arg_heading = radians(arg_heading)

    dx = round(cos(arg_heading) * arg_dist, 6)
    dy = round(sin(arg_heading) * arg_dist, 6)

    return dx, dy


def modulo_heading(arg_raw_heading):
    """Simple function to module an angle to 2*pi both fwd and backward"""
    if arg_raw_heading >= 0:
        return_value = arg_raw_heading % (2 * pi)
    else:
        return_value = -(-arg_raw_heading % (2 * pi))

    return return_value


def calculate_transformation_matrices(arg_location, arg_reverse=True):
    """Taken from http://en.wikipedia.org/wiki/Transformation_matrix, this function
    calculates appropriate transformation matrices for translating between different sets of co-ordinate systems"""
    if arg_reverse:
        # used to translate the local measure to a global one
        heading = -arg_location.heading
        translation_vector_scalar = -1
    else:
        heading = arg_location.heading
        translation_vector_scalar = 1

    rotation_matrix = array([[[cos(heading), -sin(heading)],
                              [sin(heading), cos(heading)]]])
    translation_vector = translation_vector_scalar * array([arg_location.x, arg_location.y])

    return rotation_matrix, translation_vector


def gaussian(mu, sigma, x):
    """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


def distance_between_points(p0, p1):
    """distance formula between two points,
    http://stackoverflow.com/questions/5407969/distance-formula-between-two-points-in-a-list"""
    return sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2)


def look_up_reference_frame(arg_reference_frame):
    """ translates json reference frame to enum
    @param arg_reference_frame:str
    @return:PositionReferenceFrame
    """
    if arg_reference_frame == 'level':
        return_value = PositionReferenceFrame.level_plan
    elif arg_reference_frame == 'building':
        return_value = PositionReferenceFrame.building
    elif arg_reference_frame == 'room':
        return_value = PositionReferenceFrame.room_plan
    elif arg_reference_frame == 'eastingNorthing':
        return_value = PositionReferenceFrame.easting_northing
    elif arg_reference_frame == 'longLat':
        return_value = PositionReferenceFrame.longitude_latitude
    else:
        return_value = PositionReferenceFrame.na

    return return_value


def translate_map(map_data, bbox):
    """utility function to move co-ordinates in map_data to a new origin in top left corner of bounding box"""
    x1, y1, x2, y2 = bbox
    map_data['properties']['originalBBox'] = bbox
    map_data['properties']['imageSize'] = [y2 - y1, x2 - x1, map_data['properties']['imageSize'][2]]

    for feature in map_data['features']:
        if 'bbox' in feature.keys():
            if feature['properties']['geomType'] == 'wall':
                feature['bbox'] = [0, 0, x2 - x1, y2 - y1]
            else:
                feature['bbox'] = [feature['bbox'][0] - x1, feature['bbox'][1] - y1, feature['bbox'][2] - x1,
                                   feature['bbox'][3] - y1]

        if feature['geometry']['type'] == 'Polygon':
            for i, linearring in enumerate(feature['geometry']['coordinates']):
                feature['geometry']['coordinates'][i] = [[x[0] - x1, x[1] - y1] for x in linearring]
        elif feature['geometry']['type'] == 'MultiPolygon':
            for i, polygon in enumerate(feature['geometry']['coordinates']):
                for j, linearring in enumerate(polygon):
                    feature['geometry']['coordinates'][i][j] = [[x[0] - x1, x[1] - y1] for x in linearring]
    return map_data


def create_intersection_polygon(wall_polygon, bbox_poly):
    """Takes two linearrings and returns a geoJson representation of intersection polygon"""
    return_value = {}

    my_shapely_bbox = Polygon(bbox_poly)
    my_shapely_walls = Polygon(wall_polygon[0]) # Exterior of wall Polygon only.
    my_shapely_room_walls = my_shapely_bbox.intersection(my_shapely_walls)

    if isinstance(my_shapely_room_walls, shapely.geometry.multipolygon.MultiPolygon):
        return_value = {'type': 'MultiPolygon', 'coordinates': []}
        for my_shapely_poly in my_shapely_room_walls:
            return_value['coordinates'].append([[list(map(int, a)) for a in my_shapely_poly.exterior.coords]])

    else:
        return_value['type'] = 'Polygon'
        return_value['coordinates'] = [[list(map(int, a)) for a in my_shapely_room_walls.exterior.coords]]

    return return_value


def extract_linearring_from_geojson(features):
    """function takes GeoJson Object and extracts the linearrings, they are flattened
    into exterior rings  [ [ [x1,y1],...[xn,yn] ],[ [x1,y1],...[xn,yn] ] ]"""
    multi_linearring = []
    if isinstance(features, list):
        for feature in features:

            if feature['geometry']['type'] == 'MultiPolygon':

                for feature_linearring in feature['geometry']['coordinates']:
                    multi_linearring.append(feature_linearring)

            elif feature['geometry']['type'] == 'Polygon':
                multi_linearring.append(feature['geometry']['coordinates'])

            else:
                raise ValueError('Feature Linearring not as expected')
    else:
        if isinstance(features, MultiPolygon):
            for poly in features:
                multi_linearring.append(poly.exterior.coords[:])
        else:
            multi_linearring.append(features.exterior.coords[:])

    return multi_linearring


def extract_room_from_floor_plan(map_data, room):
    """extract room geoJSON object for specified room with origin set to top left of bounding box of room
    from geoJSON dict object representing floor plan.
    @rtype : dict
    """
    room_return_value = {}
    return_value = map_data
    return_value['properties']['name'] = room

    try:
        room_return_value['features'] = [map_data['features'][[i for i, f in enumerate(map_data['features']) if
                                                               f['properties']['name'] == room][0]]]

        #Get bounding box details for floor and create a polygon linearring
        x1, y1, x2, y2 = room_return_value['features'][0]['bbox']

        return_value['properties']['position'] = {'x': x1, 'y': y1, 'heading': 0, 'referenceFrame': 'level'}

        bbox_poly = [[x1, y1], [x1, y2], [x2, y2], [x2, y1]]

        #cnt(room_data['walls']['geometry']['coordinates']).tolist()[0]

        #get walls for room and translate to new origin bbox = (0,0)
        walls = map_data['features'][
            [i for i, f in enumerate(map_data['features']) if f['properties']['name'] == 'walls'][0]]

        if walls['geometry']['type'] == 'MultiPolygon':
            multi_walls = []
            for i, wall_polygon in enumerate(walls['geometry']['coordinates']):
                multi_walls.append(
                    create_intersection_polygon(walls['geometry']['coordinates'][i], bbox_poly)[
                        'coordinates'])
            walls['geometry']['coordinates'] = multi_walls

        elif walls['geometry']['type'] == 'Polygon':
            walls['geometry'] = create_intersection_polygon(walls['geometry']['coordinates'], bbox_poly)

        else:
            raise ValueError('Walls not Polygon as expected')

        room_return_value['features'].append(walls)
        return_value['features'] = room_return_value['features']

    except IndexError:
        logging.error("Index error:" + str(sys.exc_info()[0]) + ' - ' + str(sys.exc_info()[1]))
        return_value = {}
    except:
        logging.error("Unexpected error: %s, %s" ,str(sys.exc_info()[0]) ,str(sys.exc_info()[1]))
        return_value = {}

    return return_value



