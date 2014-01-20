#!/usr/bin/env python
import abc

from pydispatch import dispatcher


"""
Base class for measurements models
"""

__author__ = 'jim'
__version__ = '0.0.1'


class MeasurementModel(object):
    """Base class for the Measurement"""

    def __init__(self,arg_name, arg_model_type,arg_subscriptions):
        self.name = arg_name
        self.location_measurement = None
        self.model_type = arg_model_type
        if arg_subscriptions is not None:
            self.subscribe_to_measurement_events(arg_subscriptions)

    @abc.abstractmethod
    def calibrate_measurement_model(self,arg_model_params):
        """Method sets up measurement model"""

    @abc.abstractmethod
    def create_location_measurement(self, signal):
        """function returns a location measurement reading
        based on the sensor_data submitted broadcasts sensor
        data"""



    def _publish_location_measurement(self):
        dispatcher.send(self.location_measurement, self)

    def subscribe_to_measurement_events(self,arg_subscriptions):
        if isinstance(arg_subscriptions,list):
            for subscription in arg_subscriptions:
                dispatcher.connect(self.create_location_measurement,dispatcher.Any,subscription)
        else:
            dispatcher.connect(self.create_location_measurement,dispatcher.Any,arg_subscriptions)

    def disconnect_from_measurement_events(self,arg_subscriptions):
        if isinstance(arg_subscriptions,list):
            for subscription in arg_subscriptions:
                dispatcher.disconnect(self.create_location_measurement,dispatcher.Any,subscription)
        else:
            dispatcher.disconnect(self.create_location_measurement,dispatcher.Any,arg_subscriptions)
