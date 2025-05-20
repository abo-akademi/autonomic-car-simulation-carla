import carla
import ai_knowledge as data
from ai_knowledge import Status
import math
import numpy as np
from collections import deque


# Executor is responsible for moving the vehicle around
# In this implementation it only needs to match the steering and speed so that we arrive at provided waypoints
class Executor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        self.target_pos = knowledge.get_location()
        # NOTE: Removed the problematic line that was trying to set a callback

    # Update the executor at some intervals to steer the car in desired direction
    def update(self, time_elapsed):
        status = self.knowledge.get_status()
        obstacle = self.knowledge.retrieve_data('obstacle_ahead')
        if status == Status.DRIVING:
        
            if obstacle:
                print("[Executor] Obstacle detected! Switching to HEALING state.")
                self.knowledge.update_status(Status.HEALING)
                return
        
            dest = self.knowledge.get_current_destination()
            self.update_control(dest, [20], time_elapsed)  # Using default speed of 20 km/h
        
        # elif status == Status.HEALING:
            # current_location = self.knowledge.get_location()
            # print("[Planner] new path")
            # escape_destination = self.get_current_destination()
            # self.make_plan(current_location, escape_destination)
            # print("[Executor] In HEALING mode - reacting to obstacle.")
            # # Brake
            # control = carla.VehicleControl()
            # control.throttle = 0.0
            # control.brake = 1.0
            # control.steer = 0.0
            # control.hand_brake = False
            # self.vehicle.apply_control(control)

    def update_control(self, destination, additional_vars, delta_time):
        

        # status = self.knowledge.get_status()
        # print(status)
        # obstacle = self.knowledge.retrieve_data('obstacle_ahead')
        # print(obstacle)

        # if obstacle:
        #     self.knowledge.update_status(Status.HEALING)
        #     return
        # elif status == Status.HEALING:
        #     # Brake
        #     control = carla.VehicleControl()
        #     control.throttle = 0.0
        #     control.brake = 1.0
        #     control.steer = 0.0
        #     control.hand_brake = False
        #     self.vehicle.apply_control(control)

        # Get current vehicle transform
        current_transform = self.vehicle.get_transform()
        current_location = current_transform.location
        current_rotation = current_transform.rotation

        # Calculate direction vector to destination
        direction = carla.Vector3D(
            destination.x - current_location.x,
            destination.y - current_location.y,
            0.0  # Ignore z-axis for steering calculation
        )

        # Get the vehicle's forward vector
        forward = current_rotation.get_forward_vector()

        # Normalize vectors for dot product calculation
        direction_length = (direction.x ** 2 + direction.y ** 2) ** 0.5

        # Avoid division by zero
        if direction_length > 0.0:
            direction.x /= direction_length
            direction.y /= direction_length
        else:
            # We've reached the destination
            direction = forward  # Just maintain current direction

        # Calculate dot product for angle
        dot_product = forward.x * direction.x + forward.y * direction.y
        # Clamp dot_product to [-1, 1] to avoid math domain errors
        dot_product = max(-1.0, min(1.0, dot_product))

        # Calculate the angle using arccos of dot product
        angle = math.acos(dot_product)

        # Calculate cross product to determine direction of the angle
        cross_product = forward.x * direction.y - forward.y * direction.x

        # Adjust angle based on cross product sign
        if cross_product < 0:
            angle = -angle

        # Create control command
        control = carla.VehicleControl()

        # Calculate steering (normalized to [-1, 1])
        # The steering formula needs tuning based on vehicle and environment
        steering_amount = angle / (math.pi / 2)  # Adjust sensitivity
        control.steer = max(-1.0, min(1.0, steering_amount))  # Clamp to valid range

        # Get target speed, default to 20 if not specified
        target_speed = 20.0  # Default speed in km/h
        if len(additional_vars) > 0:
            target_speed = additional_vars[0]

        # Calculate current speed in km/h
        velocity = self.vehicle.get_velocity()
        current_speed = 3.6 * (velocity.x ** 2 + velocity.y ** 2) ** 0.5  # Convert to km/h

        # Calculate distance to destination
        distance = self.knowledge.distance(current_location, destination)

        # Adjust throttle based on distance and speed
        if distance > 5.0:  # If not close to destination
            if current_speed < target_speed:
                control.throttle = 0.7  # Accelerate
                control.brake = 0.0
            else:
                control.throttle = 0.0
                control.brake = 0.3  # Slow down if over target speed
        else:
            # When close to destination, slow down
            slow_factor = max(0.1, min(1.0, distance / 5.0))
            control.throttle = 0.5 * slow_factor
            if current_speed > 5.0:  # Apply brake if still moving fast
                control.brake = 0.5
            else:
                control.brake = 0.0

        control.hand_brake = False

        # Apply control to vehicle
        self.vehicle.apply_control(control)


# Planner is responsible for creating a plan for moving around
# In our case it creates a list of waypoints to follow so that vehicle arrives at destination
# Alternatively this can also provide a list of waypoints to try avoid crashing or 'uncrash' itself
class Planner(object):
    def __init__(self, knowledge, vehicle):
        self.knowledge = knowledge
        self.vehicle = vehicle
        self.path = deque([])

    # Create a map of waypoints to follow to the destination and save it
    def make_plan(self, source, destination):
        self.path = self.build_path(source, destination)
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

    # Update internal state to make sure that there are waypoints to follow and that we have not arrived yet
    def update_plan(self):
        status = self.knowledge.get_status()

        if len(self.path) == 0:
            return

        if self.knowledge.arrived_at(self.path[0]):
            self.path.popleft()

        if len(self.path) == 0:
            self.knowledge.update_status(Status.ARRIVED)
        
        if status == Status.HEALING:
            current_location = self.knowledge.get_location()
            print("[Planner] new path")
            escape_destination = self.get_current_destination()
            self.make_plan(current_location, escape_destination)

        else:
            self.knowledge.update_status(Status.DRIVING)

    # get current destination
    def get_current_destination(self):
        status = self.knowledge.get_status()
        # if we are driving, then the current destination is next waypoint
        if status == Status.DRIVING:
            # TODO: Take into account traffic lights and other cars
            return self.path[0]
        if status == Status.ARRIVED:
            return self.knowledge.get_location()
        if status == Status.HEALING:
            print("[Planner] new path")

            current_location = self.knowledge.get_location()
            waypoint = self.vehicle.get_world().get_map().get_waypoint(current_location)

            options = waypoint.next(2.0)  # next possible waypoint

            if waypoint.lane_change == carla.LaneChange.Right:
                options.append(waypoint.get_right_lane())
            elif waypoint.lane_change == carla.LaneChange.Left:
                options.append(waypoint.get_left_lane())

            # only available waypoint
            escape_route = [wp.transform.location for wp in options if wp and self.is_path_clear(wp)]

            if escape_route:
                return escape_route[0]
            
            return self.knowledge.get_location()
        if status == Status.CRASHED:
            # TODO: implement function for crash handling, should provide map of wayoints to move towards to for exiting crash state.
            # You should use separate waypoint list for that, to not mess with the original path.
            return self.knowledge.get_location()
        # otherwise destination is same as current position
        return self.knowledge.get_location()
    
    def is_path_clear(self, waypoint):
        lidar_points = self.knowledge.retrieve_data('lidar_points')
        if not lidar_points:
            return True 

        points = np.array(lidar_points)

        mask = (np.abs(points[:, 0] - waypoint.transform.location.x) < 3) & \
            (np.abs(points[:, 1] - waypoint.transform.location.y) < 3)
        
        return points[mask].shape[0] < 5


    # TODO: Implementation
    def build_path(self, source, destination):
        self.path = deque([])
        self.path.append(destination)
        # TODO: create path of waypoints from source to destination
        return self.path