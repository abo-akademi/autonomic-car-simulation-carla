import weakref
import carla
import ai_knowledge as data


# Monitor is responsible for reading the data from the sensors and telling it to the knowledge
class Monitor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        weak_self = weakref.ref(self)

        self.knowledge.update_data('location', self.vehicle.get_transform().location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)

        # Setup lane invasion sensor
        world = self.vehicle.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.lane_detector = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
        self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))

        # Uncomment this section if you want to add LIDAR later and correctly implement _on_lidar_data
        # # Setup LIDAR sensor
        # lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        # lidar_bp.set_attribute('range', '50')  # 50 meters range
        # lidar_bp.set_attribute('rotation_frequency', '10')  # 10 Hz rotation frequency
        # lidar_bp.set_attribute('channels', '32')  # 32 channels
        # lidar_bp.set_attribute('points_per_second', '100000')  # 100k points per second
        # lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.0))  # Place on top of the car
        # self.lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        # self.lidar_sensor.listen(lambda data: Monitor._on_lidar_data(weak_self, data))

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        # Update the position of vehicle into knowledge
        self.knowledge.update_data('location', self.vehicle.get_transform().location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.knowledge.update_data('lane_invasion', event.crossed_lane_markings)

    # Uncomment and implement this if you want to use LIDAR data
    # @staticmethod
    # def _on_lidar_data(weak_self, data):
    #   self = weak_self()
    #   if not self:
    #     return
    #   # LIDAR data processing needs a different approach
    #   # LidarDetection doesn't have a direct 'location' attribute
    #   lidar_data = np.frombuffer(data.raw_data, dtype=np.float32).reshape([-1, 3])
    #   self.knowledge.update_data('lidar_points', lidar_data)


# Analyser is responsible for parsing all the data that the knowledge has received from Monitor and turning it into something usable
class Analyser(object):
    def __init__(self, knowledge):
        self.knowledge = knowledge

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        return