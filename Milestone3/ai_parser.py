import weakref
import carla
import ai_knowledge as data
import numpy as np


# Monitor is responsible for reading the data from the sensors and telling it to the knowledge
class Monitor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        weak_self = weakref.ref(self)

        # Store the vehicle in knowledge so other components can use it
        self.knowledge.update_data('vehicle', self.vehicle)
        self.knowledge.update_data('location', self.vehicle.get_transform().location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)

        # Setup lane invasion sensor
        world = self.vehicle.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.lane_detector = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
        self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))

        # Setup LIDAR sensor for Milestone 2
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '50')  # 50 meters range
        lidar_bp.set_attribute('rotation_frequency', '10')  # 10 Hz rotation frequency
        lidar_bp.set_attribute('channels', '32')  # 32 channels
        lidar_bp.set_attribute('points_per_second', '100000')  # 100k points per second
        lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.0))  # Place on top of the car
        self.lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        self.lidar_sensor.listen(lambda data: Monitor._on_lidar_data(weak_self, data))

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        # Update the position of vehicle into knowledge
        self.knowledge.update_data('location', self.vehicle.get_transform().location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)

        # For Milestone 3: Check if the vehicle is at traffic lights
        traffic_light = self.vehicle.get_traffic_light()
        if traffic_light is not None:
            # If the traffic light is red, set at_lights to True
            at_red_light = traffic_light.get_state() == carla.TrafficLightState.Red
            self.knowledge.update_data('at_lights', at_red_light)
            self.knowledge.update_data('traffic_light', traffic_light)
        else:
            self.knowledge.update_data('at_lights', False)
            self.knowledge.update_data('traffic_light', None)

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.knowledge.update_data('lane_invasion', event.crossed_lane_markings)

    @staticmethod
    def _on_lidar_data(weak_self, data):
        self = weak_self()
        if not self:
            return
        # LIDAR data processing for obstacle detection
        # Carla's LIDAR data has 4 float values per point: (x, y, z, intensity)
        lidar_data = np.frombuffer(data.raw_data, dtype=np.float32).reshape([-1, 4])
        # Just keep the xyz coordinates
        points_xyz = lidar_data[:, :3]
        self.knowledge.update_data('lidar_points', points_xyz)


# Analyser is responsible for parsing all the data that the knowledge has received from Monitor and turning it into something usable
class Analyser(object):
    def __init__(self, knowledge):
        self.knowledge = knowledge
        # Set default target speed for Milestone 3
        self.knowledge.update_data('target_speed', 44)  # 44 km/h as specified

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        # For Milestone 3: Traffic light handling
        at_lights = self.knowledge.retrieve_data('at_lights')
        traffic_light = self.knowledge.retrieve_data('traffic_light')

        # Always ensure target_speed is set
        if self.knowledge.retrieve_data('target_speed') is None:
            self.knowledge.update_data('target_speed', 44)  # Default to 44 km/h as specified

        if at_lights and traffic_light is not None:
            # If at red light, set target speed to 0
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                self.knowledge.update_data('target_speed', 0)
            else:
                # Not red, restore speed
                self.knowledge.update_data('target_speed', 44)
        else:
            # Not at lights, ensure proper speed
            current_speed = self.knowledge.retrieve_data('target_speed')
            if current_speed == 0:  # Only restore if it was previously set to 0
                self.knowledge.update_data('target_speed', 44)

        # For Milestone 2: Obstacle detection using LIDAR data
        # This will be more fully implemented in Milestone 2, but we're setting up the framework here
        lidar_points = self.knowledge.retrieve_data('lidar_points')
        if lidar_points is not None:
            # This part will be expanded in Milestone 2
            pass