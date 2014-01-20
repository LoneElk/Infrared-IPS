#!/usr/bin/env python
"""
Module holds building/map objects based on the
ontology presented in http://www.geos.ed.ac.uk/~gisteac/eeo_agi/2011-12/3_worboys_21102011.pdf
"""
__author__ = 'jim'
__version__ = '0.0.1'

import copy

from shapely.geometry import Polygon

from globals.global_constants import *
from globals.utility_functions import look_up_reference_frame, \
    translate_map, extract_linearring_from_geojson,extract_room_from_floor_plan

from globals.position import Position

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-Building Classes=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

class Container(object):
    """Base class for Rooms and Levels which are an aggregation of rooms"""
    def __init__(self, arg_container_type, arg_container_geojson_obj):
        self.name = None
        self.container_type = arg_container_type
        self._container_geojson_obj = None
        self.floor_linearring = None
        self.floor = None
        self._floors_geojson = None
        self.scale = None
        self.wall_linearring = None
        self.walls = None
        self._walls_geojson = None
        self.usable_area = None
        self.usable_area_linearring = None
        self.position =None
        self.container_geojson_obj = arg_container_geojson_obj

    @property
    def container_geojson_obj(self):
        return self._container_geojson_obj

    @container_geojson_obj.setter
    def container_geojson_obj(self,arg_container_geojson_obj):
        self._container_geojson_obj = arg_container_geojson_obj
        self._set_container_attributes()

    def _set_container_attributes(self):
        """extract polygon data for room with origin set to top left of bounding box"""
        self.scale = self._container_geojson_obj['properties']['scale']
        self._walls_geojson = [
            self.container_geojson_obj['features'][j] for j in
            [i for i,f in enumerate(self.container_geojson_obj['features'])
             if f['properties']['geomType'] == wall_const]]

        self._floors_geojson = [
            self.container_geojson_obj['features'][j] for j in
            [i for i,f in enumerate(self.container_geojson_obj['features'])
             if f['properties']['geomType'] == floor_const]]

        self.name = self._container_geojson_obj['properties']['name']

        #Get Linearring for Walls and Floors

        self.floor_linearring = extract_linearring_from_geojson(self._floors_geojson)
        self.wall_linearring = extract_linearring_from_geojson(self._walls_geojson)

        self.floor = Floor(self.floor_linearring)
        self.walls = Wall(self.wall_linearring)

        self.usable_area_linearring = extract_linearring_from_geojson(self.floor.polygon.difference(
            self.walls.polygon.buffer(wall_thickness_const*self.scale/2.)))
        self.usable_area = Surface(SurfaceType.abstract,[self.usable_area_linearring],True)

        self.position = Position(arg_x=self._container_geojson_obj['properties']['position']['x'],
                                 arg_y=self._container_geojson_obj['properties']['position']['y'],
                                 arg_heading=self._container_geojson_obj['properties']['position']['heading'],
                                 arg_reference_frame=look_up_reference_frame(
                                     self._container_geojson_obj['properties']['position']['referenceFrame']),
                                 arg_dataum=Datum.NA,
                                 arg_reference_frame_name=self._container_geojson_obj['properties']['name'])

class Room(Container):
    def __init__(self, arg_room_geojson_obj):
        super(Room, self).__init__(ContainerType.room, arg_room_geojson_obj)
        pass


class Level(Container):
    def __init__(self, arg_room_geojson_obj):
        super(Level, self).__init__(ContainerType.level, arg_room_geojson_obj)

        self.rooms = {}

        #extract geojson for each room
        for room in [r['properties']['name'] for r in self._floors_geojson]:
            # Need to create a new object to be passed to the get_room_details function to prevent the original
            # map_data object being manipulated by the function using a different name.
            # As dictionaries are mutable http://python.net/~goodger/projects/ ->
            # pycon/2007/idiomatic/handout.html#other-languages-have-variables

            room_data = extract_room_from_floor_plan(copy.deepcopy(self._container_geojson_obj),room)

            if room_data:
                bbox = room_data['features'][[i for i,f in enumerate(room_data['features']) if f['properties']['name'] == room ][0]]['bbox']
                room_data = translate_map(copy.deepcopy(room_data),bbox)
                self.rooms[room]=Room(room_data)


#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=--=-=-= Building Geometry Elements -=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Note Walls modelled as Polygon as interested in polygon representation rather than lines as per indoor-json

class BuildlingPolygon(object):
    """base geometry class creates polygons from linearrings
    Not support from lines or points at the moment"""
    def __init__(self, arg_geom_type, arg_linearring, arg_accessible):
        self.geom_type = arg_geom_type
        self._linearring = None
        self.polygon = None
        self.linearring = arg_linearring
        self.accessible = arg_accessible

    @property
    def linearring(self):
        return self._linearring

    @linearring.setter
    def linearring(self, arg_linearring):
        self._linearring = arg_linearring

        self.polygon = None

        for linearring in self._linearring:
            #1st argument exterior second interior - we shouldn't have any of these, yet
            poly = Polygon(linearring[0],linearring[1:])
            if self.polygon is None:
                self.polygon = poly
            else:
                self.polygon = self.polygon.union(poly)

class Surface(BuildlingPolygon):
    def __init__(self,arg_surface_type, arg_linearring, arg_accessible):
        super(Surface,self).__init__(GeomType.surface,arg_linearring, arg_accessible)
        self.surface_type = arg_surface_type


class Barrier(BuildlingPolygon):
    def __init__(self,arg_surface_type, arg_linearring, arg_accessible):
        super(Barrier,self).__init__(GeomType.barrier,arg_linearring, arg_accessible)
        self.surface_type = arg_surface_type

class Floor(Surface):
    def __init__(self, arg_linearring):
        super(Floor, self).__init__(SurfaceType.floor, arg_linearring, True)

class Wall(Surface):
    def __init__(self, arg_linearring):
        super(Wall, self).__init__(SurfaceType.wall, arg_linearring, False)


