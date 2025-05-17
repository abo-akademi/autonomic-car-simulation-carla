#!/usr/bin/env python

import glob
import os
import sys
import weakref

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import ai_knowledge as data

class Monitor(object):
    """ Monitors vehicle sensors and updates AI knowledge. """
    
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        weak_self = weakref.ref(self)
        
        self.knowledge.update_data('location', self.vehicle.get_transform().location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)

        world = self.vehicle.get_world()
        self._setup_sensors(world, weak_self)

    def _setup_sensors(self, world, weak_self):
        """ Initializes vehicle sensors and attaches callback handlers. """
        
        # Lane invasion sensor
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.lane_detector = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
        self.lane_detector.listen(lambda event: self._on_invasion(weak_self, event))

        # Lidar sensor
        bp_lidar = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        self.lidar_sensor = world.spawn_actor(bp_lidar, carla.Transform(), attach_to=self.vehicle)
        self.lidar_sensor.listen(lambda data: self._on_lidar_data(weak_self, data))

        # Depth camera sensor
        bp_depth = world.get_blueprint_library().find('sensor.camera.depth')
        self.depth_sensor = world.spawn_actor(bp_depth, carla.Transform(), attach_to=self.vehicle)
        self.depth_sensor.listen(lambda image: self._on_depth_data(weak_self, image))

    def update(self, time_elapsed):
        """ Updates AI state with vehicle positioning and traffic light status. """
        self.knowledge.update_data('location', self.vehicle.get_transform().location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)

        # Check traffic light state
        traffic_light = self.vehicle.get_traffic_light()
        if traffic_light:
            self.knowledge.update_data('traffic_light_state', traffic_light.get_state())

    @staticmethod
    def _on_invasion(weak_self, event):
        """ Handles lane invasion events. """
        self = weak_self()
        if not self:
            return
        self.knowledge.update_data('lane_invasion', event.crossed_lane_markings)

    @staticmethod
    def _on_lidar_data(weak_self, data):
        """ Processes lidar data and updates AI knowledge. """
        self = weak_self()
        if not self:
            return
        points = [point.location for point in data]  # Extract lidar points
        self.knowledge.update_data('lidar_points', points)

    @staticmethod
    def _on_depth_data(weak_self, image):
        """ Processes depth camera images and updates AI knowledge. """
        self = weak_self()
        if not self:
            return
        depth_data = image.raw_data  # Extract depth image data
        self.knowledge.update_data('depth_image', depth_data)


class Analyser(object):
    """ Converts raw sensor data into usable AI navigation insights. """
    
    def __init__(self, knowledge):
        self.knowledge = knowledge
        self.knowledge.set_data_changed_callback(self.on_data_updated)

    def update(self, time_elapsed):
        """ Processes AI knowledge and extracts useful navigation data. """
        location = self.knowledge.retrieve_data('location')
        lidar_points = self.knowledge.retrieve_data('lidar_points')
        traffic_light_state = self.knowledge.retrieve_data('traffic_light_state')

        # Detect obstacles and adjust AI pathing
        if lidar_points and any(p.distance(location) < 5.0 for p in lidar_points):
            print("Analyser: Close obstacle detected. AI should adjust route.")

        # Stop vehicle at red traffic light
        if traffic_light_state == carla.TrafficLightState.Red:
            print("Analyser: Red light detected! AI should stop.")
            self.knowledge.update_status(data.Status.ARRIVED)  # Stop AI movement

    def on_data_updated(self, data_name, value):
        """ Handles dynamic updates from AI sensors. """
        print(f"Analyser: Data updated -> {data_name}: {value}")
