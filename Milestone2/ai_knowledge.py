#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from enum import Enum

class Status(Enum):
    ARRIVED = 0  
    DRIVING = 1
    CRASHED = 2
    HEALING = 3
    UNDEFINED = 4

class Knowledge(object):
    """ Class that holds knowledge of AI's current state and interaction points. """
    
    def __init__(self):
        self.status = Status.ARRIVED
        self.memory = {'location': carla.Vector3D(0.0, 0.0, 0.0)}
        self.destination = self.get_location()

        # Callback functions
        self.status_changed = lambda *_, **__: None
        self.destination_changed = lambda *_, **__: None
        self.data_changed = lambda *_, **__: None

    def set_data_changed_callback(self, callback):
        """ Registers a callback function for when data updates. """
        if callable(callback):
            self.data_changed = callback

    def set_status_changed_callback(self, callback):
        """ Registers a callback function for when status updates. """
        if callable(callback):
            self.status_changed = callback

    def set_destination_changed_callback(self, callback):
        """ Registers a callback function for when destination changes. """
        if callable(callback):
            self.destination_changed = callback

    def get_status(self):
        """ Retrieves the current AI status. """
        return self.status

    def set_status(self, new_status):
        """ Updates AI status. """
        self.status = new_status

    def get_current_destination(self):
        """ Retrieves the current destination. """
        return self.destination

    def retrieve_data(self, data_name):
        """ Retrieves data safely from memory. """
        return self.memory.get(data_name, None)

    def update_status(self, new_status):
        """ Updates AI status while ensuring proper transitions. """
        if self.status == Status.CRASHED and new_status == Status.HEALING:
            self.set_status(new_status)
            self.status_changed(new_status)
        elif self.status != new_status:
            self.set_status(new_status)
            self.status_changed(new_status)

    def get_location(self):
        """ Returns current vehicle location. """
        return self.retrieve_data('location') or carla.Vector3D(0.0, 0.0, 0.0)

    def arrived_at(self, destination):
        """ Checks if the vehicle has arrived at the destination. """
        return self.distance(self.get_location(), destination) < 5.0

    def update_destination(self, new_destination):
        """ Updates the AI’s destination safely. """
        if self.distance(self.destination, new_destination) >= 5.0 and self.destination != new_destination:
            self.destination = new_destination
            self.destination_changed(new_destination)

    def update_data(self, data_name, pars):
        """ Stores new data while triggering callbacks. """
        self.memory[data_name] = pars
        if callable(self.data_changed):
            self.data_changed(data_name, pars)

    def distance(self, vec1, vec2):
        """ Computes the distance between two points. """
        l1 = carla.Location(vec1)
        l2 = carla.Location(vec2)
        return l1.distance(l2)
