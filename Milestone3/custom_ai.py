#!/usr/bin/env python

import glob
import os
import sys
import argparse
import random
import time

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import custom_ai as ai
import ai_knowledge as data
import ai_control as control
import ai_parser as parser

class Autopilot(object):
    """ Central AI module handling vehicle automation. """
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.knowledge = data.Knowledge()
        self.knowledge.set_status_changed_callback(self.status_updated)
        self.analyser = parser.Analyser(self.knowledge)
        self.monitor = parser.Monitor(self.knowledge, self.vehicle)
        self.planner = control.Planner(self.knowledge)
        self.executor = control.Executor(self.knowledge, self.vehicle)
        self.prev_time = time.monotonic()
        self.route_finished = lambda *_, **__: None
        self.crashed = lambda *_, **__: None

    def status_updated(self, new_status):
        """ Callback for AI status changes. """
        if new_status == data.Status.ARRIVED and callable(self.route_finished):
            self.route_finished(self)
        elif new_status == data.Status.CRASHED and callable(self.crashed):
            self.crashed(self)
        elif new_status == data.Status.HEALING:
            print("Autopilot: AI is healing from a crash.")

    def set_route_finished_callback(self, callback):
        """ Sets the route completion callback. """
        if callable(callback):
            self.route_finished = callback

    def set_crash_callback(self, callback):
        """ Sets the crash event callback. """
        if callable(callback):
            self.crashed = callback

    def get_vehicle(self):
        """ Returns the assigned vehicle. """
        return self.vehicle
    
    def update(self):
        """ Updates all AI modules and returns the current status. """
        try:
            current_time = time.monotonic()
            delta_time = current_time - self.prev_time
            self.prev_time = current_time

            self.monitor.update(delta_time)
            self.analyser.update(delta_time)
            self.planner.update(delta_time)
            self.executor.update(delta_time)

            return self.knowledge.get_status()
        except Exception as e:
            print(f"Autopilot Update Error: {e}")
            return self.knowledge.get_status()

    def set_destination(self, destination):
        """ Defines AI's next destination safely. """
        if self.knowledge.get_status() != data.Status.DRIVING:
            self.planner.make_plan(self.vehicle.get_transform(), destination)
            self.knowledge.update_destination(destination)
