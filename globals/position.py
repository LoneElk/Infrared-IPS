#!/usr/bin/env python

"""
Module holds basic location concepts such as position which are used in
"""
__author__ = 'jim'
__version__ = '0.0.1'

# ----------------- Constants -----------------
from globals.global_constants import *
from globals.utility_functions import modulo_heading

class Position(object):
    """Base position class currently supporting 2d orientation x,y,theata """

    def __init__(self, arg_x, arg_y, arg_heading, arg_reference_frame=PositionReferenceFrame.longitude_latitude,
                 arg_dataum=Datum.WGS84,arg_reference_frame_name=None):

        # Frame of reference
        self.frame_of_reference = arg_reference_frame
        self.frame_of_reference_name = arg_reference_frame_name
        self.datum = arg_dataum

        # Spatial Co-ordinates
        self.x = arg_x
        self.y = arg_y

        # Heading
        self._h = 0
        self.heading = arg_heading

    @property
    def xyh(self):
        return self.x, self.y, self.heading

    @property
    def xy(self):
        return self.x,self.y

    @property
    def heading(self):
        return self._h

    @heading.setter
    def heading(self,h):
        self._h = modulo_heading(h)

