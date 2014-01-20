__author__ = 'jim'

import unittest
import sys
import json
import logging

import globals.building_model as bm
from globals.global_constants import *


def load_map(file_name):
    """load map for specified room"""
    map_data = {}
    try:
        with open(file_name,'r') as fs:
            map_data = json.load(fs)
    except IOError as e:
        logging.error("I/O error({0}): {1}".format(e.errno, e.strerror))
    except:
        logging.error("Unexpected error: %s - %s",str(sys.exc_info()[0]), str(sys.exc_info()[1]))
    return map_data


class SurfaceUnitTests(unittest.TestCase):


    def test_surface_attributes(self):
        self.surface = bm.Surface('test', [],False)
        self.assertEquals(self.surface.surface_type,'test')
        self.assertItemsEqual(self.surface.linearring, [])

        self.surface = bm.Floor([])
        self.assertEquals(self.surface.surface_type,SurfaceType.floor)
        self.assertItemsEqual(self.surface.linearring, [])

        self.surface = bm.Wall([])
        self.assertEquals(self.surface.surface_type,SurfaceType.wall)
        self.assertItemsEqual(self.surface.linearring, [])
        self.assertTrue(self.surface.polygon is None)


        #Single Polygon no interior
        poly = [[[0,0],[0,1],[1,1],[1,0]]]
        self.surface = bm.Wall([poly])
        self.assertEquals(self.surface.surface_type,SurfaceType.wall)
        self.assertItemsEqual(self.surface.linearring, [poly])

        #Multiple Polygons
        poly2 = [[[0,0],[0,2],[2,2]]]
        self.surface = bm.Wall([poly, poly2])
        self.assertEquals(self.surface.surface_type,SurfaceType.wall)
        self.assertItemsEqual(self.surface.linearring, [poly,poly2])
        self.assertItemsEqual(self.surface.polygon.exterior.coords[:],
                              [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (2.0, 2.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0)])


class ContainerTests(unittest.TestCase):

    def setUp(self):
        self.container_geojson_obj = {"type": "FeatureCollection",
                                      "features": [
                                          {"geometry": {"type": "Polygon",
                                                        "coordinates": [
                                                            [[539, 2], [282, 1],
                                                             [266, 120], [252, 94],
                                                             [125, 96], [117, 13],
                                                             [0, 14], [1, 301],
                                                             [124, 300], [127, 231],
                                                             [540, 233]]]},
                                           "type": "Feature",
                                           "properties": {"geomType": "floor",
                                                          "accessible": True,
                                                          "colour": [79, 233, 252],
                                                          "name": "hall",
                                                          "level": 0},
                                           "bbox": [0, 0, 542, 303]}, {
                                              "geometry": {"type": "MultiPolygon",
                                                           "coordinates": [
                                                               [[
                                                                    [0, 0],
                                                                    [0, 35],
                                                                    [1, 35],
                                                                    [3, 118],
                                                                    [18, 118],
                                                                    [18, 34],
                                                                    [97, 33],
                                                                    [101, 117],
                                                                    [297, 114],
                                                                    [304, 18],
                                                                    [515, 22],
                                                                    [516, 113],
                                                                    [542, 121],
                                                                    [542, 0],
                                                                    [0, 0]
                                                                ]],
                                                               [[
                                                                    [100, 303],
                                                                    [125, 303],
                                                                    [125, 236],
                                                                    [394, 232],
                                                                    [394, 207],
                                                                    [101, 207],
                                                                    [99, 277],
                                                                    [21, 276],
                                                                    [21, 209],
                                                                    [4, 209],
                                                                    [2, 293],
                                                                    [101, 299],
                                                                    [100, 303]
                                                                ]],
                                                               [[
                                                                    [542, 233],
                                                                    [542, 194],
                                                                    [483, 209],
                                                                    [483, 232],
                                                                    [542, 233]
                                                                ]]
                                                           ]},
                                              "type": "Feature",
                                              "properties": {"geomType": "wall",
                                                             "accessible": False,
                                                             "colour": [0, 0, 0],
                                                             "name": "walls",
                                                             "level": 0},
                                              "bbox": [0, 0, 542, 303]}],
                                      "properties": {
                                          "srcImage": "/Users/jim/Dropbox/Documents/Msc/Thesis/"
                                                      "A4/Infrared-IPS/location_rules_engine/"
                                                      "utilities/floor_plan/floorplan1.jpg",
                                          "scale": 100, "originalBBox": [617, 16, 1159, 319],
                                          "name": "hall", "imageSize": [303, 542, 3],
                                          'position': {'y': 16, 'x': 721, 'referenceFrame': 'level', 'heading': 0}}}


        self.container_geojson_obj2 = {"type": "FeatureCollection", "features": [
            {"geometry": {"type": "Polygon", "coordinates": [[[1, 2], [2, 450], [525, 451], [526, 4]]]},
             "type": "Feature",
             "properties": {"geomType": "floor", "accessible": True, "colour": [186, 187, 248], "name": "livingRoom",
                            "level": 0}, "bbox": [0, 0, 531, 452]}, {"geometry": {"type": "Polygon", "coordinates": [
                [[0, 0], [0, 450], [517, 450], [530, 437], [530, 11], [517, 0], [379, 0], [379, 22], [505, 25],
                 [505, 425], [21, 422], [21, 26], [290, 22], [290, 0], [0, 0]]]}, "type": "Feature",
                                                                     "properties": {"geomType": "wall",
                                                                                    "accessible": False,
                                                                                    "colour": [0, 0, 0],
                                                                                    "name": "walls", "level": 0},
                                                                     "bbox": [0, 0, 542, 303]}], "properties": {
        "srcImage": "/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/location_rules_engine/utilities/floor_plan/floorplan1.jpg",
        "scale": 100, "originalBBox": [721, 226, 1252, 678], "name": "livingRoom",
        "imageSize": [452, 531, 3],
        'position': {'y': 16, 'x': 721, 'referenceFrame': 'level', 'heading': 0}
         }}

        self.floor_plan_geojson_file = '/Users/jim/Dropbox/Documents/Msc/Thesis/A4/Infrared-IPS/configuration/floorplan1.json'

    def test_container_attributes(self):
        # Multipolygon
        self.container = bm.Container(ContainerType.room, self.container_geojson_obj)
        self.assertEqual(self.container.name,self.container_geojson_obj['properties']['name'])
        self.assertEquals(self.container.container_type, ContainerType.room)
        self.assertEquals(self.container.container_geojson_obj, self.container_geojson_obj)
        self.assertItemsEqual(self.container.floor_linearring,
                              [self.container_geojson_obj['features'][0]['geometry']['coordinates']])
        self.assertItemsEqual(self.container.wall_linearring,
                              self.container_geojson_obj['features'][1]['geometry']['coordinates'])

        self.assertItemsEqual(self.container.floor.polygon.exterior.coords[:],
                          [(539.0, 2.0), (282.0, 1.0), (266.0, 120.0), (252.0, 94.0), (125.0, 96.0),
                           (117.0, 13.0), (0.0, 14.0), (1.0, 301.0),
                           (124.0, 300.0), (127.0, 231.0), (540.0, 233.0), (539.0, 2.0)])

        #Polygon
        self.container = bm.Container(ContainerType.room, self.container_geojson_obj2)
        self.assertEqual(self.container.name,self.container_geojson_obj2['properties']['name'])
        self.assertItemsEqual(self.container.floor_linearring,
                              [self.container_geojson_obj2['features'][0]['geometry']['coordinates']])
        self.assertItemsEqual(self.container.wall_linearring,
                              [self.container_geojson_obj2['features'][1]['geometry']['coordinates']])

        self.assertItemsEqual(self.container.floor.polygon.exterior.coords[:],
                         [(1.0, 2.0), (2.0, 450.0), (525.0, 451.0), (526.0, 4.0), (1.0, 2.0)])

        #Floor Plan
        self.container = bm.Container(ContainerType.room, load_map(self.floor_plan_geojson_file))
        # print self.container.floor.polygon.exterior.coords[:]
        # print self.container.walls.polygon.exterior.coords[:]
        # print self.container.usable_area.polygon

    def test_room_attributes(self):
        self.room = bm.Room(self.container_geojson_obj)
        self.assertEqual(self.room.name, self.container_geojson_obj['properties']['name'])
        self.assertEquals(self.room.container_type, ContainerType.room)
        self.assertEquals(self.room.container_geojson_obj, self.container_geojson_obj)
        self.room = bm.Room(load_map(self.floor_plan_geojson_file))

    def test_level_attributes(self):
        self.level = bm.Level(self.container_geojson_obj)
        self.level = bm.Level(load_map(self.floor_plan_geojson_file))

if __name__ == '__main__':
    unittest.main()
