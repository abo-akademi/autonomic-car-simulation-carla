#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
import math
import numpy as np

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import ai_knowledge as data
from ai_knowledge import Status

# Executor is responsible for moving the vehicle around
# In this implementation it only needs to match the steering and speed so that we arrive at provided waypoints
# BONUS TODO: implement different speed limits so that planner would also provide speed target speed in addition to direction
class Executor(object):
  def __init__(self, knowledge, vehicle, speed_limit=30):
    self.vehicle = vehicle
    self.knowledge = knowledge
    self.target_pos = knowledge.get_location()
    #define speed limit
    self.speed_limit = speed_limit

    # Register callback functions to react to knowledge updates
    self.knowledge.set_status_changed_callback(self.on_status_changed)
    self.knowledge.set_data_changed_callback(self.on_data_changed)

    
  #Update the executor at some intervals to steer the car in desired direction
  def update(self, time_elapsed):
    status = self.knowledge.get_status()
    #TODO: this needs to be able to handle
    if status == Status.DRIVING:
      dest = self.knowledge.get_current_destination()
      self.update_control(dest, [1], time_elapsed)

    #Callback function for status updates
    def on_status_changed(self,new_status):
        print(f"Executor: Vehicle status changed to {new_status}")
        if new_status == Status.CRASHED:
            print("Executor: Handling crash event...")
            self.knowledge.update_status(Status.HEALING)  # Start recovery mode
 

  # TODO: steer in the direction of destination and throttle or brake depending on how close we are to destination
  # TODO: Take into account that exiting the crash site could also be done in reverse, so there might need to be additional data passed between planner and executor, or there needs to be some way to tell this that it is ok to drive in reverse during HEALING and CRASHED states. An example is additional_vars, that could be a list with parameters that can tell us which things we can do (for example going in reverse)
  def update_control(self, destination, additional_vars, delta_time):
    velocity = self.vehicle.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    #calculate throttle and heading
    control = carla.VehicleControl()

    # Retrieve target speed from additional_vars or default AI speed limit
    target_speed = additional_vars[0] if additional_vars else self.speed_limit

    if speed<target_speed:
        control.throttle = 0.5
        control.brake = 0.0
    else:  
        control.brake = 0.0
        control.hand_brake = False

    # Reverse Handling **(Proper Placement)**
    if self.knowledge.get_status() in [Status.CRASHED, Status.HEALING]:
        control.reverse = True  # Enable reverse when needed
    else:
        control.reverse = False  # Move forward normally
    
    # Get vehicle's current location and heading direction
    current_location = self.vehicle.get_transform().location
    forward_vector = self.vehicle.get_transform().get_forward_vector()
    current_rotation = self.vehicle.get_transform().rotation.yaw  # Vehicle's current facing direction

    # Compute the vector toward the destination
    target_vector = carla.Vector3D(destination.x - current_location.x,destination.y - current_location.y,.0)  # Ignoring Z for simplicity

    # Extract desired rotation at destination
    target_rotation = current_rotation
    if isinstance(additional_vars, dict):
      target_rotation = additional_vars.get('target_rotation', current_rotation)

    def safe_normalize(vec):
        norm = np.linalg.norm(vec)
        return vec if norm == 0 else vec / norm

    v1 = safe_normalize(np.array([forward_vector.x, forward_vector.y]))
    v2 = safe_normalize(np.array([target_vector.x, target_vector.y]))

    # Compute dot product and angle
    dot_product = np.dot(v1, v2)
    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Clip to avoid domain errors

    # Determine turn direction using cross product
    cross_product = np.cross(v1, v2)
    steer_direction = np.sign(cross_product)  # +1 for left, -1 for right

    # Adjust steering based on vehicle's current and target rotation
    rotation_difference = (target_rotation - current_rotation) % 360
    if rotation_difference > 180:
        rotation_difference -= 360  # Normalize to [-180, 180] for better handling

    control.steer = max(-1.0, min(1.0, 0.9 * control.steer + 0.1 * (steer_direction * angle + rotation_difference / 90)))
    control.hand_brake = False

    self.vehicle.apply_control(control)

     # Check if the vehicle has arrived at its destination **and** is facing the correct way
    if self.knowledge.arrived_at(destination) and abs(rotation_difference) < 5.0:  # Acceptable error range
        self.knowledge.update_status(Status.ARRIVED)
        print("Exercise route finished!")
    # Callback function for data updates
    def on_data_changed(self, data_name, value):
        print(f"Executor: Data updated -> {data_name}: {value}")

    
# Planner is responsible for creating a plan for moving around
# In our case it creates a list of waypoints to follow so that vehicle arrives at destination
# Alternatively this can also provide a list of waypoints to try avoid crashing or 'uncrash' itself
class Planner(object):
  def __init__(self, knowledge):
    self.knowledge = knowledge
    self.path = deque([])
    #Register callback for destination changes
    self.knowledge.set_destination_changed_callback(self.on_destination_changed) 

   
  # Create a map of waypoints to follow to the destination and save it
  def make_plan(self, source, destination):
    self.path = self.build_path(source,destination)
    self.update_plan()
    self.knowledge.update_destination(self.get_current_destination())
  
  # Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    self.update_plan()
    self.knowledge.update_destination(self.get_current_destination())
  
  #Update internal state to make sure that there are waypoints to follow and that we have not arrived yet
  def update_plan(self):
    if len(self.path) == 0:
       return
    
    if self.knowledge.arrived_at(self.path[0]):
        self.path.popleft()
    
    if len(self.path) == 0:
        self.knowledge.update_status(Status.ARRIVED)
        print("Planner: Vehicle has arrived at destination.")
    else:
        self.knowledge.update_status(Status.DRIVING)
        print("Planner: Vehicle continues driving toward target.")

# Callback function for destination changes
  def on_destination_changed(self, new_destination):
    print(f"Planner: Destination updated -> {new_destination}")
    self.make_plan(self.knowledge.get_location(), new_destination)

  def find_exit_route(self):
    print("Planner: Generating temporary exit route from crash site...")
    return self.knowledge.get_location()
    
  #get current destination 
  def get_current_destination(self):
    status = self.knowledge.get_status()
    #if we are driving, then the current destination is next waypoint
    if status == Status.DRIVING:
      #TODO: Take into account traffic lights and other cars
        return self.path[0] if len(self.path) > 0 else self.knowledge.get_location()

    if status in [Status.CRASHED, Status.HEALING]:
        return self.find_exit_route()

    # if status == Status.ARRIVED:
    #   return self.knowledge.get_location()
    # if status == Status.HEALING:
    #   #TODO: Implement crash handling. Probably needs to be done by following waypoint list to exit the crash site.
    #   #Afterwards needs to remake the path.
    #   return self.knowledge.get_location()
    # if status == Status.CRASHED:
    #   #TODO: implement function for crash handling, should provide map of wayoints to move towards to for exiting crash state. 
    #   #You should use separate waypoint list for that, to not mess with the original path. 
    #   return self.knowledge.get_location()
    # #otherwise destination is same as current position
    # return self.knowledge.get_location()

  #TODO: Implementation
  def build_path(self, source, destination):
    self.path = deque([])
    world = self.knowledge.get_world()
    map = world.get_map()
    source_wp = map.get_waypoint(source)
    destination_wp = map.get_waypoint(destination)

    current_wp = source_wp
    while current_wp.transform.location.distance(destination_wp.transform.location) > 2.0:
        next_wps = current_wp.next(5.0)
        if not next_wps:
            break
        current_wp = next_wps[0]
        self.path.append(current_wp.transform.location)


    self.path.append(destination)
    #TODO: create path of waypoints from source to destination
    return self.path


