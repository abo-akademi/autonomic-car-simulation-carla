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
        # Store the vehicle in knowledge if not already there
        if not self.knowledge.retrieve_data('vehicle'):
            self.knowledge.update_data('vehicle', self.vehicle)

    # Update the executor at some intervals to steer the car in desired direction
    def update(self, time_elapsed):
        status = self.knowledge.get_status()
        if status == Status.DRIVING:
            dest = self.knowledge.get_current_destination()
            # Get target speed from knowledge
            target_speed = self.knowledge.retrieve_data('target_speed')
            if target_speed is None:
                target_speed = 20  # Default speed if not set
                self.knowledge.update_data('target_speed', target_speed)
            self.update_control(dest, [target_speed], time_elapsed)
        elif status == Status.HEALING:
            # Implement obstacle avoidance behavior
            # In Milestone 3, we're implementing this before Milestone 2,
            # but we'll add a placeholder for now
            dest = self.knowledge.get_current_destination()
            target_speed = self.knowledge.retrieve_data('target_speed')
            if target_speed is None:
                target_speed = 10  # Slower speed during avoidance
            self.update_control(dest, [target_speed], time_elapsed)

    def update_control(self, destination, additional_vars, delta_time):
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

        # If at traffic lights and they are red, stop
        at_lights = self.knowledge.retrieve_data('at_lights')
        if at_lights:
            target_speed = 0.0

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
    def __init__(self, knowledge):
        self.knowledge = knowledge
        self.path = deque([])
        self.world_map = None

    # Create a map of waypoints to follow to the destination and save it
    def make_plan(self, source, destination):
        # Get the world map from the vehicle
        self.world_map = self.knowledge.retrieve_data('vehicle').get_world().get_map()

        self.path = self.build_path(source, destination)
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

    # Update internal state to make sure that there are waypoints to follow and that we have not arrived yet
    def update_plan(self):
        if len(self.path) == 0:
            return

        if self.knowledge.arrived_at(self.path[0]):
            self.path.popleft()

        if len(self.path) == 0:
            self.knowledge.update_status(Status.ARRIVED)
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
            # TODO: Implement crash handling. Probably needs to be done by following waypoint list to exit the crash site.
            # Afterwards needs to remake the path.
            return self.knowledge.get_location()
        if status == Status.CRASHED:
            # TODO: implement function for crash handling, should provide map of wayoints to move towards to for exiting crash state.
            # You should use separate waypoint list for that, to not mess with the original path.
            return self.knowledge.get_location()
        # otherwise destination is same as current position
        return self.knowledge.get_location()

    # Implementation for Milestone 3: Building a path using waypoints
    def build_path(self, source, destination):
        path = deque([])

        if self.world_map is None:
            # Fallback if world_map is not available
            vehicle = self.knowledge.retrieve_data('vehicle')
            if vehicle:
                self.world_map = vehicle.get_world().get_map()
            else:
                print("Warning: No vehicle available to get world map")
                path.append(destination)
                return path

        # Get waypoint objects for source and destination
        source_location = source.location if hasattr(source, 'location') else source
        source_waypoint = self.world_map.get_waypoint(source_location)
        destination_waypoint = self.world_map.get_waypoint(carla.Location(destination))

        # Add destination to the path (we'll work backwards to this)
        path.appendleft(destination)

        # Current waypoint to build path from
        current_waypoint = source_waypoint

        # Set a maximum path length to prevent infinite loops
        max_iterations = 1000
        iterations = 0

        # Keep track of waypoints to determine when we're getting closer
        previous_distance = float('inf')

        while iterations < max_iterations:
            iterations += 1

            # Get possible next waypoints
            next_waypoints = current_waypoint.next(5.0)  # 5.0 meters ahead

            if not next_waypoints:
                break

            # Default to first waypoint if no better option
            best_waypoint = next_waypoints[0]
            min_distance = float('inf')

            # Find the waypoint that gets us closer to destination
            for waypoint in next_waypoints:
                dist = waypoint.transform.location.distance(destination_waypoint.transform.location)
                if dist < min_distance:
                    min_distance = dist
                    best_waypoint = waypoint

            # Consider lane changes if they get us closer
            if current_waypoint.lane_change in [carla.LaneChange.Left, carla.LaneChange.Both]:
                left_lane = current_waypoint.get_left_lane()
                if left_lane:
                    left_dist = left_lane.transform.location.distance(destination_waypoint.transform.location)
                    if left_dist < min_distance:
                        min_distance = left_dist
                        best_waypoint = left_lane

            if current_waypoint.lane_change in [carla.LaneChange.Right, carla.LaneChange.Both]:
                right_lane = current_waypoint.get_right_lane()
                if right_lane:
                    right_dist = right_lane.transform.location.distance(destination_waypoint.transform.location)
                    if right_dist < min_distance:
                        min_distance = right_dist
                        best_waypoint = right_lane

            # Add the waypoint location to our path
            path.appendleft(best_waypoint.transform.location)

            # Update current waypoint
            current_waypoint = best_waypoint

            # Check if we're making progress
            if min_distance >= previous_distance and iterations > 10:
                # If we're not getting closer after several iterations, we might be stuck
                # Try to find a direct path to destination
                direct_path = self.world_map.get_waypoint_xodr(destination_waypoint.road_id,
                                                               destination_waypoint.lane_id, destination_waypoint.s)
                if direct_path:
                    path.appendleft(direct_path.transform.location)
                break

            previous_distance = min_distance

            # Check if we're close enough to destination
            if min_distance < 5.0:
                break

        return path