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

def main():
    """ Spawns AI-driven NPC vehicles into the simulation. """
    
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port')
    argparser.add_argument('-n', '--number-of-vehicles', metavar='N', default=10, type=int, help='Vehicle count')
    argparser.add_argument('-d', '--delay', metavar='D', default=2.0, type=float, help='Spawn delay')
    argparser.add_argument('--safe', action='store_true', help='Avoid accident-prone vehicles')
    args = argparser.parse_args()

    actor_list = []
    vai_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)

    try:
        world = client.get_world()
        world = client.load_world('Town03')
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]

        def route_finished(autopilot):
            """ Assigns a new destination upon arrival. """
            print("Vehicle arrived at destination")
            new_destination = random.choice(world.get_map().get_spawn_points())
            autopilot.set_destination(new_destination)

        def try_spawn_random_vehicle_at(transform):
            """ Spawns a vehicle at a given location. """
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                blueprint.set_attribute('color', random.choice(blueprint.get_attribute('color').recommended_values))
            blueprint.set_attribute('role_name', 'autopilot')

            vehicle = world.try_spawn_actor(blueprint, transform)
            if vehicle is not None:
                actor_list.append(vehicle)
                autopilot = ai.Autopilot(vehicle)
                autopilot.set_route_finished_callback(route_finished)
                vai_list.append(autopilot)
                print(f"Spawned {vehicle.type_id} at {transform.location}")
                return True
            return False

        spawn_points = list(world.get_map().get_spawn_points())
        random.shuffle(spawn_points)
        print(f"Found {len(spawn_points)} spawn points.")

        count = args.number_of_vehicles

        for spawn_point in spawn_points:
            if try_spawn_random_vehicle_at(spawn_point):
                count -= 1
            if count <= 0:
                break

        print(f"Spawned {args.number_of_vehicles} vehicles, press Ctrl+C to exit.")

        # Parallel processing for better performance
        while True:
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage
            for controller in vai_list:
                controller.update()

    finally:
        print(f"\nDestroying {len(actor_list)} actors")
        client.apply_batch([carla.command.DestroyActor(x.id) for x in actor_list])

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nDone.")
