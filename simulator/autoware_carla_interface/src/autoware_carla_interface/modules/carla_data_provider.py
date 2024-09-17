# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.sr/bin/env python

"""Modified CARLA Data Provider from CARLA scenario runner."""

from __future__ import print_function

import datetime
import math
import re
import threading

import carla
from numpy import random
from six import iteritems


def calculate_velocity(actor):
    """Calculate the velocity of a actor."""
    velocity_squared = actor.get_velocity().x ** 2
    velocity_squared += actor.get_velocity().y ** 2
    return math.sqrt(velocity_squared)


class CarlaDataProvider(object):  # pylint: disable=too-many-public-methods
    _actor_velocity_map = {}
    _actor_location_map = {}
    _actor_transform_map = {}
    _traffic_light_map = {}
    _carla_actor_pool = {}
    _global_osc_parameters = {}
    _client = None
    _world = None
    _map = None
    _sync_flag = False
    _spawn_points = None
    _spawn_index = 0
    _blueprint_library = None
    _all_actors = None
    _ego_vehicle_route = None
    _traffic_manager_port = 8000
    _random_seed = 2000
    _rng = random.RandomState(_random_seed)
    _local_planner = None
    _runtime_init_flag = False
    _lock = threading.Lock()

    @staticmethod
    def set_local_planner(plan):
        CarlaDataProvider._local_planner = plan

    @staticmethod
    def get_local_planner():
        return CarlaDataProvider._local_planner

    @staticmethod
    def register_actor(actor, transform=None):
        """Add new actor to dictionaries."""
        with CarlaDataProvider._lock:
            if actor in CarlaDataProvider._actor_velocity_map:
                raise KeyError(
                    "Vehicle '{}' already registered. Cannot register twice!".format(actor.id)
                )
            else:
                CarlaDataProvider._actor_velocity_map[actor] = 0.0
            if actor in CarlaDataProvider._actor_location_map:
                raise KeyError(
                    "Vehicle '{}' already registered. Cannot register twice!".format(actor.id)
                )
            elif transform:
                CarlaDataProvider._actor_location_map[actor] = transform.location
            else:
                CarlaDataProvider._actor_location_map[actor] = None

            if actor in CarlaDataProvider._actor_transform_map:
                raise KeyError(
                    "Vehicle '{}' already registered. Cannot register twice!".format(actor.id)
                )
            else:
                CarlaDataProvider._actor_transform_map[actor] = transform

    @staticmethod
    def update_osc_global_params(parameters):
        """Updates/initializes global osc parameters."""
        CarlaDataProvider._global_osc_parameters.update(parameters)

    @staticmethod
    def get_osc_global_param_value(ref):
        """Return updated global osc parameter value."""
        return CarlaDataProvider._global_osc_parameters.get(ref.replace("$", ""))

    @staticmethod
    def register_actors(actors, transforms=None):
        """Add new set of actors to dictionaries."""
        if transforms is None:
            transforms = [None] * len(actors)

        for actor, transform in zip(actors, transforms):
            CarlaDataProvider.register_actor(actor, transform)

    @staticmethod
    def on_carla_tick():
        with CarlaDataProvider._lock:
            for actor in CarlaDataProvider._actor_velocity_map:
                if actor is not None and actor.is_alive:
                    CarlaDataProvider._actor_velocity_map[actor] = calculate_velocity(actor)

            for actor in CarlaDataProvider._actor_location_map:
                if actor is not None and actor.is_alive:
                    CarlaDataProvider._actor_location_map[actor] = actor.get_location()

            for actor in CarlaDataProvider._actor_transform_map:
                if actor is not None and actor.is_alive:
                    CarlaDataProvider._actor_transform_map[actor] = actor.get_transform()

            world = CarlaDataProvider._world
            if world is None:
                print("WARNING: CarlaDataProvider couldn't find the world")

            CarlaDataProvider._all_actors = None

    @staticmethod
    def get_velocity(actor):
        """Return the absolute velocity for the given actor."""
        for key in CarlaDataProvider._actor_velocity_map:
            if key.id == actor.id:
                return CarlaDataProvider._actor_velocity_map[key]

        # We are intentionally not throwing here
        # This may cause exception loops in py_trees
        print("{}.get_velocity: {} not found!".format(__name__, actor))
        return 0.0

    @staticmethod
    def get_location(actor):
        """Return the location for the given actor."""
        for key in CarlaDataProvider._actor_location_map:
            if key.id == actor.id:
                return CarlaDataProvider._actor_location_map[key]

        # We are intentionally not throwing here
        # This may cause exception loops in py_trees
        print("{}.get_location: {} not found!".format(__name__, actor))
        return None

    @staticmethod
    def get_transform(actor):
        """Return the transform for the given actor."""
        for key in CarlaDataProvider._actor_transform_map:
            if key.id == actor.id:
                # The velocity location information is the entire behavior tree updated every tick
                # The ego vehicle is created before the behavior tree tick, so exception handling needs to be added
                if CarlaDataProvider._actor_transform_map[key] is None:
                    return actor.get_transform()
                return CarlaDataProvider._actor_transform_map[key]

        # We are intentionally not throwing here
        # This may cause exception loops in py_trees
        print("{}.get_transform: {} not found!".format(__name__, actor))
        return None

    @staticmethod
    def get_random_seed():
        """Return the random seed."""
        return CarlaDataProvider._rng

    @staticmethod
    def set_client(client):
        """Set the CARLA client."""
        CarlaDataProvider._client = client

    @staticmethod
    def get_client():
        """Get the CARLA client."""
        return CarlaDataProvider._client

    @staticmethod
    def set_world(world):
        """Set the world and world settings."""
        CarlaDataProvider._world = world
        CarlaDataProvider._sync_flag = world.get_settings().synchronous_mode
        CarlaDataProvider._map = world.get_map()
        CarlaDataProvider._blueprint_library = world.get_blueprint_library()
        CarlaDataProvider.generate_spawn_points()
        CarlaDataProvider.prepare_map()

    @staticmethod
    def get_world():
        """Return world."""
        return CarlaDataProvider._world

    @staticmethod
    def get_map(world=None):
        """Get the current map."""
        if CarlaDataProvider._map is None:
            if world is None:
                if CarlaDataProvider._world is None:
                    raise ValueError("class member 'world'' not initialized yet")
                else:
                    CarlaDataProvider._map = CarlaDataProvider._world.get_map()
            else:
                CarlaDataProvider._map = world.get_map()

        return CarlaDataProvider._map

    @staticmethod
    def get_all_actors():
        """Return all the world actors."""
        if CarlaDataProvider._all_actors:
            return CarlaDataProvider._all_actors

        CarlaDataProvider._all_actors = CarlaDataProvider._world.get_actors()
        return CarlaDataProvider._all_actors

    @staticmethod
    def set_runtime_init_mode(flag):
        """Set the runtime init mode."""
        CarlaDataProvider._runtime_init_flag = flag

    @staticmethod
    def is_runtime_init_mode():
        """Return true if runtime init mode is used."""
        return CarlaDataProvider._runtime_init_flag

    @staticmethod
    def find_weather_presets():
        """Get weather presets from CARLA."""
        rgx = re.compile(".+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)")

        def format_name(x):
            return " ".join(m.group(0) for m in rgx.finditer(x))

        presets = [x for x in dir(carla.WeatherParameters) if re.match("[A-Z].+", x)]
        return [(getattr(carla.WeatherParameters, x), format_name(x)) for x in presets]

    @staticmethod
    def prepare_map():
        """Set the current map and loads all traffic lights for this map to_traffic_light_map."""
        if CarlaDataProvider._map is None:
            CarlaDataProvider._map = CarlaDataProvider._world.get_map()

        # Parse all traffic lights
        CarlaDataProvider._traffic_light_map.clear()
        for traffic_light in CarlaDataProvider._world.get_actors().filter("*traffic_light*"):
            if traffic_light not in list(CarlaDataProvider._traffic_light_map):
                CarlaDataProvider._traffic_light_map[traffic_light] = traffic_light.get_transform()
            else:
                raise KeyError(
                    "Traffic light '{}' already registered. Cannot register twice!".format(
                        traffic_light.id
                    )
                )

    @staticmethod
    def generate_spawn_points():
        """Generate spawn points for the current map."""
        spawn_points = list(CarlaDataProvider.get_map(CarlaDataProvider._world).get_spawn_points())
        CarlaDataProvider._rng.shuffle(spawn_points)
        CarlaDataProvider._spawn_points = spawn_points
        CarlaDataProvider._spawn_index = 0

    @staticmethod
    def check_road_length(wp, length: float):
        waypoint_separation = 5

        cur_len = 0
        road_id, lane_id = wp.road_id, wp.lane_id
        while True:
            wps = wp.next(waypoint_separation)
            # The same road_id and lane_id，judged to be in the same section to be tested
            next_wp = None
            for p in wps:
                if p.road_id == road_id and p.lane_id == lane_id:
                    next_wp = p
                    break
            if next_wp is None:
                break
            cur_len += waypoint_separation
            if cur_len >= length:
                return True
            wp = next_wp
        return False

    @staticmethod
    def get_road_lanes(wp):
        if wp.is_junction:
            return []
        # find the most left lane's waypoint

        lane_id_set = set()
        pre_left = wp
        while wp and wp.lane_type == carla.LaneType.Driving:
            if wp.lane_id in lane_id_set:
                break
            lane_id_set.add(wp.lane_id)

            # carla bug: get_left_lane Return error，and never Return none. It's a infinite loop.
            pre_left = wp
            wp = wp.get_left_lane()

        # # Store data from the left lane to the right lane
        # # list<key, value>, key=laneid, value=waypoint
        lane_list = []
        lane_id_set.clear()
        wp = pre_left
        while wp and wp.lane_type == carla.LaneType.Driving:
            if wp.lane_id in lane_id_set:
                break
            lane_id_set.add(wp.lane_id)

            lane_list.append(wp)

            # carla bug: Return error, never return none, endless loop
            wp = wp.get_right_lane()

        return lane_list

    @staticmethod
    def get_road_lane_cnt(wp):
        lanes = CarlaDataProvider.get_road_lanes(wp)
        return len(lanes)

    @staticmethod
    def get_waypoint_by_laneid(lane_num: int):
        if CarlaDataProvider._spawn_points is None:
            CarlaDataProvider.generate_spawn_points()

        if CarlaDataProvider._spawn_index >= len(CarlaDataProvider._spawn_points):
            print("No more spawn points to use")
            return None
        else:
            pos = CarlaDataProvider._spawn_points[
                CarlaDataProvider._spawn_index
            ]  # pylint: disable=unsubscriptable-object
            CarlaDataProvider._spawn_index += 1
            wp = CarlaDataProvider.get_map().get_waypoint(
                pos.location, project_to_road=True, lane_type=carla.LaneType.Driving
            )

            road_lanes = CarlaDataProvider.get_road_lanes(wp)

            lane = int(float(lane_num))
            if lane > len(road_lanes):
                return None
            else:
                return road_lanes[lane - 1]

    # cspell:ignore rolename
    @staticmethod
    def create_blueprint(
        model, rolename="scenario", color=None, actor_category="car", attribute_filter=None
    ):
        """Set up the blueprint of an actor given its model and other relevant parameters."""

        def check_attribute_value(blueprint, name, value):
            """Check if the blueprint has that attribute with that value."""
            if not blueprint.has_attribute(name):
                return False

            attribute_type = blueprint.get_attribute(key).type
            if attribute_type == carla.ActorAttributeType.Bool:
                return blueprint.get_attribute(name).as_bool() == value
            elif attribute_type == carla.ActorAttributeType.Int:
                return blueprint.get_attribute(name).as_int() == value
            elif attribute_type == carla.ActorAttributeType.Float:
                return blueprint.get_attribute(name).as_float() == value
            elif attribute_type == carla.ActorAttributeType.String:
                return blueprint.get_attribute(name).as_str() == value

            return False

        # cspell:ignore carlacola carlamotors
        _actor_blueprint_categories = {
            "car": "vehicle.tesla.model3",
            "van": "vehicle.volkswagen.t2",
            "truck": "vehicle.carlamotors.carlacola",
            "trailer": "",
            "semitrailer": "",
            "bus": "vehicle.volkswagen.t2",
            "motorbike": "vehicle.kawasaki.ninja",
            "bicycle": "vehicle.diamondback.century",
            "train": "",
            "tram": "",
            "pedestrian": "walker.pedestrian.0001",
        }

        # Set the model
        try:
            blueprints = CarlaDataProvider._blueprint_library.filter(model)
            if attribute_filter is not None:
                for key, value in attribute_filter.items():
                    blueprints = [x for x in blueprints if check_attribute_value(x, key, value)]

            blueprint = CarlaDataProvider._rng.choice(blueprints)
        except ValueError:
            # The model is not part of the blueprint library. Let's take a default one for the given category
            bp_filter = "vehicle.*"
            new_model = _actor_blueprint_categories[actor_category]
            if new_model != "":
                bp_filter = new_model
            print(
                "WARNING: Actor model {} not available. Using instead {}".format(model, new_model)
            )
            blueprint = CarlaDataProvider._rng.choice(
                CarlaDataProvider._blueprint_library.filter(bp_filter)
            )

        # Set the color
        if color:
            if not blueprint.has_attribute("color"):
                print(
                    "WARNING: Cannot set Color ({}) for actor {} due to missing blueprint attribute".format(
                        color, blueprint.id
                    )
                )
            else:
                default_color_rgba = blueprint.get_attribute("color").as_color()
                default_color = "({}, {}, {})".format(
                    default_color_rgba.r, default_color_rgba.g, default_color_rgba.b
                )
                try:
                    blueprint.set_attribute("color", color)
                except ValueError:
                    # Color can't be set for this vehicle
                    print(
                        "WARNING: Color ({}) cannot be set for actor {}. Using instead: ({})".format(
                            color, blueprint.id, default_color
                        )
                    )
                    blueprint.set_attribute("color", default_color)
        else:
            if blueprint.has_attribute("color") and rolename != "hero":
                color = CarlaDataProvider._rng.choice(
                    blueprint.get_attribute("color").recommended_values
                )
                blueprint.set_attribute("color", color)

        # Make pedestrians mortal
        if blueprint.has_attribute("is_invincible"):
            blueprint.set_attribute("is_invincible", "false")

        # Set the rolename
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", rolename)

        return blueprint

    @staticmethod
    def handle_actor_batch(batch, tick=True):
        """Forward a CARLA command batch to spawn actors to CARLA, and gather the responses."""
        sync_mode = CarlaDataProvider.is_sync_mode()
        actors = []

        if CarlaDataProvider._client:
            responses = CarlaDataProvider._client.apply_batch_sync(batch, sync_mode and tick)
        else:
            raise ValueError("class member 'client'' not initialized yet")

        # Wait (or not) for the actors to be spawned properly before we do anything
        if not tick:
            pass
        elif CarlaDataProvider.is_runtime_init_mode():
            CarlaDataProvider._world.wait_for_tick()
        elif sync_mode:
            CarlaDataProvider._world.tick()
        else:
            CarlaDataProvider._world.wait_for_tick()

        actor_ids = [r.actor_id for r in responses if not r.error]
        for r in responses:
            if r.error:
                print("WARNING: Not all actors were spawned")
                break
        actors = list(CarlaDataProvider._world.get_actors(actor_ids))
        return actors

    @staticmethod
    def request_new_actor(
        model,
        spawn_point,
        rolename="scenario",
        autopilot=False,
        random_location=False,
        color=None,
        actor_category="car",
        attribute_filter=None,
        tick=True,
    ):
        """Create a new actor, returning it if successful (None otherwise)."""
        blueprint = CarlaDataProvider.create_blueprint(
            model, rolename, color, actor_category, attribute_filter
        )

        if random_location:
            actor = None
            while not actor:
                spawn_point = CarlaDataProvider._rng.choice(CarlaDataProvider._spawn_points)
                actor = CarlaDataProvider._world.try_spawn_actor(blueprint, spawn_point)

        else:
            # For non prop models, slightly lift the actor to avoid collisions with the ground
            z_offset = 0.2 if "prop" not in model else 0

            # DO NOT USE spawn_point directly, as this will modify spawn_point permanently
            _spawn_point = carla.Transform(carla.Location(), spawn_point.rotation)
            _spawn_point.location.x = spawn_point.location.x
            _spawn_point.location.y = spawn_point.location.y
            _spawn_point.location.z = spawn_point.location.z + z_offset
            actor = CarlaDataProvider._world.try_spawn_actor(blueprint, _spawn_point)

        if actor is None:
            print(
                "WARNING: Cannot spawn actor {} at position {}".format(model, spawn_point.location)
            )
            return None

        # De/activate the autopilot of the actor if it belongs to vehicle
        if autopilot:
            if isinstance(actor, carla.Vehicle):
                actor.set_autopilot(autopilot, CarlaDataProvider._traffic_manager_port)
            else:
                print("WARNING: Tried to set the autopilot of a non vehicle actor")

        # Wait for the actor to be spawned properly before we do anything
        if not tick:
            pass
        elif CarlaDataProvider.is_runtime_init_mode():
            CarlaDataProvider._world.wait_for_tick()
        elif CarlaDataProvider.is_sync_mode():
            CarlaDataProvider._world.tick()
        else:
            CarlaDataProvider._world.wait_for_tick()

        if actor is None:
            return None

        CarlaDataProvider._carla_actor_pool[actor.id] = actor
        CarlaDataProvider.register_actor(actor, spawn_point)
        return actor

    @staticmethod
    def request_new_actors(actor_list, attribute_filter=None, tick=True):
        """Series of actor in batch. If this was successful, the new actors are returned, None otherwise."""
        SpawnActor = carla.command.SpawnActor  # pylint: disable=invalid-name
        PhysicsCommand = carla.command.SetSimulatePhysics  # pylint: disable=invalid-name
        FutureActor = carla.command.FutureActor  # pylint: disable=invalid-name
        ApplyTransform = carla.command.ApplyTransform  # pylint: disable=invalid-name
        SetAutopilot = carla.command.SetAutopilot  # pylint: disable=invalid-name
        SetVehicleLightState = carla.command.SetVehicleLightState  # pylint: disable=invalid-name

        batch = []
        actors = []

        CarlaDataProvider.generate_spawn_points()

        for actor in actor_list:
            # Get the blueprint
            blueprint = CarlaDataProvider.create_blueprint(
                actor.model, actor.rolename, actor.color, actor.category, attribute_filter
            )

            # Get the spawn point
            transform = actor.transform
            if not transform:
                continue
            if actor.random_location:
                if CarlaDataProvider._spawn_index >= len(CarlaDataProvider._spawn_points):
                    print("No more spawn points to use")
                    break
                else:
                    _spawn_point = CarlaDataProvider._spawn_points[
                        CarlaDataProvider._spawn_index
                    ]  # pylint: disable=unsubscriptable-object
                    CarlaDataProvider._spawn_index += 1

            else:
                _spawn_point = carla.Transform()
                _spawn_point.rotation = transform.rotation
                _spawn_point.location.x = transform.location.x
                _spawn_point.location.y = transform.location.y
                if blueprint.has_tag("walker"):
                    # On imported OpenDRIVE maps, spawning of pedestrians can fail.
                    # By increasing the z-value the chances of success are increased.
                    map_name = CarlaDataProvider._map.name.split("/")[-1]
                    if not map_name.startswith("OpenDrive"):
                        _spawn_point.location.z = transform.location.z + 0.2
                    else:
                        _spawn_point.location.z = transform.location.z + 0.8
                else:
                    _spawn_point.location.z = transform.location.z + 0.2

            # Get the command
            command = SpawnActor(blueprint, _spawn_point)
            command.then(
                SetAutopilot(FutureActor, actor.autopilot, CarlaDataProvider._traffic_manager_port)
            )

            if (
                actor.args is not None
                and "physics" in actor.args
                and actor.args["physics"] == "off"
            ):
                command.then(ApplyTransform(FutureActor, _spawn_point)).then(
                    PhysicsCommand(FutureActor, False)
                )
            elif actor.category == "misc":
                command.then(PhysicsCommand(FutureActor, True))
            if actor.args is not None and "lights" in actor.args and actor.args["lights"] == "on":
                command.then(SetVehicleLightState(FutureActor, carla.VehicleLightState.All))

            batch.append(command)

        actors = CarlaDataProvider.handle_actor_batch(batch, tick)

        if not actors:
            return None

        for actor in actors:
            if actor is None:
                continue
            CarlaDataProvider._carla_actor_pool[actor.id] = actor
            CarlaDataProvider.register_actor(actor, _spawn_point)

        return actors

    @staticmethod
    def request_new_batch_actors(
        model,
        amount,
        spawn_points,
        autopilot=False,
        random_location=False,
        rolename="scenario",
        attribute_filter=None,
        tick=True,
    ):
        SpawnActor = carla.command.SpawnActor  # pylint: disable=invalid-name
        SetAutopilot = carla.command.SetAutopilot  # pylint: disable=invalid-name
        FutureActor = carla.command.FutureActor  # pylint: disable=invalid-name

        CarlaDataProvider.generate_spawn_points()

        batch = []

        for i in range(amount):
            # Get vehicle by model
            blueprint = CarlaDataProvider.create_blueprint(
                model, rolename, attribute_filter=attribute_filter
            )

            if random_location:
                if CarlaDataProvider._spawn_index >= len(CarlaDataProvider._spawn_points):
                    print(
                        "No more spawn points to use. Spawned {} actors out of {}".format(
                            i + 1, amount
                        )
                    )
                    break
                else:
                    spawn_point = CarlaDataProvider._spawn_points[
                        CarlaDataProvider._spawn_index
                    ]  # pylint: disable=unsubscriptable-object
                    CarlaDataProvider._spawn_index += 1
            else:
                try:
                    spawn_point = spawn_points[i]
                except IndexError:
                    print("The amount of spawn points is lower than the amount of vehicles spawned")
                    break

            if spawn_point:
                batch.append(
                    SpawnActor(blueprint, spawn_point).then(
                        SetAutopilot(
                            FutureActor, autopilot, CarlaDataProvider._traffic_manager_port
                        )
                    )
                )

        actors = CarlaDataProvider.handle_actor_batch(batch, tick)
        for actor in actors:
            if actor is None:
                continue
            CarlaDataProvider._carla_actor_pool[actor.id] = actor
            CarlaDataProvider.register_actor(actor, spawn_point)

        return actors

    @staticmethod
    def get_actors():
        """Return list of actors and their ids."""
        return iteritems(CarlaDataProvider._carla_actor_pool)

    @staticmethod
    def actor_id_exists(actor_id):
        """Check if a certain id is still at the simulation."""
        if actor_id in CarlaDataProvider._carla_actor_pool:
            return True

        return False

    @staticmethod
    def get_hero_actor():
        """Get the actor object of the hero actor if it exists, Return none otherwise."""
        for actor_id in CarlaDataProvider._carla_actor_pool:
            if CarlaDataProvider._carla_actor_pool[actor_id].attributes["role_name"] == "hero":
                return CarlaDataProvider._carla_actor_pool[actor_id]
        return None

    @staticmethod
    def get_actor_by_id(actor_id):
        """Get an actor from the pool by using its ID. If the actor does not exist, None is returned."""
        print(CarlaDataProvider._carla_actor_pool)
        if actor_id in CarlaDataProvider._carla_actor_pool:
            return CarlaDataProvider._carla_actor_pool[actor_id]

        print("Non-existing actor id {}".format(actor_id))
        return None

    @staticmethod
    def get_actor_by_name(role_name: str):
        for actor_id in CarlaDataProvider._carla_actor_pool:
            if CarlaDataProvider._carla_actor_pool[actor_id].attributes["role_name"] == role_name:
                return CarlaDataProvider._carla_actor_pool[actor_id]
        print(f"Non-existing actor name {role_name}")
        return None

    @staticmethod
    def remove_actor_by_id(actor_id):
        """Remove an actor from the pool using its ID."""
        if actor_id in CarlaDataProvider._carla_actor_pool:
            CarlaDataProvider._carla_actor_pool[actor_id].destroy()
            CarlaDataProvider._carla_actor_pool[actor_id] = None
            CarlaDataProvider._carla_actor_pool.pop(actor_id)
        else:
            print("Trying to remove a non-existing actor id {}".format(actor_id))

    @staticmethod
    def is_sync_mode():
        """Return true if synchronous mode is used."""
        return CarlaDataProvider._sync_flag

    @staticmethod
    def remove_actors_in_surrounding(location, distance):
        """Remove all actors from the pool that are closer than distance to the provided location."""
        for actor_id in CarlaDataProvider._carla_actor_pool.copy():
            if (
                CarlaDataProvider._carla_actor_pool[actor_id].get_location().distance(location)
                < distance
            ):
                CarlaDataProvider._carla_actor_pool[actor_id].destroy()
                CarlaDataProvider._carla_actor_pool.pop(actor_id)

        # Remove all keys with None values
        CarlaDataProvider._carla_actor_pool = {
            k: v for k, v in CarlaDataProvider._carla_actor_pool.items() if v
        }

    @staticmethod
    def get_traffic_manager_port():
        """Get the port of the traffic manager."""
        return CarlaDataProvider._traffic_manager_port

    @staticmethod
    def set_traffic_manager_port(tm_port):
        """Set the port to use for the traffic manager."""
        CarlaDataProvider._traffic_manager_port = tm_port

    @staticmethod
    def cleanup():
        """Cleanup and remove all entries from all dictionaries."""
        DestroyActor = carla.command.DestroyActor  # pylint: disable=invalid-name
        batch = []

        for actor_id in CarlaDataProvider._carla_actor_pool.copy():
            actor = CarlaDataProvider._carla_actor_pool[actor_id]
            if actor is not None and actor.is_alive:
                batch.append(DestroyActor(actor))

        if CarlaDataProvider._client:
            try:
                CarlaDataProvider._client.apply_batch_sync(batch)
            except RuntimeError as e:
                if "time-out" in str(e):
                    pass
                else:
                    raise e

        CarlaDataProvider._actor_velocity_map.clear()
        CarlaDataProvider._actor_location_map.clear()
        CarlaDataProvider._actor_transform_map.clear()
        CarlaDataProvider._traffic_light_map.clear()
        CarlaDataProvider._map = None
        CarlaDataProvider._world = None
        CarlaDataProvider._sync_flag = False
        CarlaDataProvider._ego_vehicle_route = None
        CarlaDataProvider._all_actors = None
        CarlaDataProvider._carla_actor_pool = {}
        CarlaDataProvider._client = None
        CarlaDataProvider._spawn_points = None
        CarlaDataProvider._spawn_index = 0
        CarlaDataProvider._rng = random.RandomState(CarlaDataProvider._random_seed)
        CarlaDataProvider._runtime_init_flag = False

    @property
    def world(self):
        return self._world


class GameTime(object):
    """Provides access to the CARLA game time."""

    _current_game_time = 0.0  # Elapsed game time after starting this Timer
    _carla_time = 0.0
    _last_frame = 0
    _platform_timestamp = 0
    _init = False

    @staticmethod
    def on_carla_tick(timestamp):
        """Handle the callback receiving the CARLA time. Update time only when the frame is more recent than the last frame."""
        if GameTime._last_frame < timestamp.frame:
            frames = timestamp.frame - GameTime._last_frame if GameTime._init else 1
            GameTime._current_game_time += timestamp.delta_seconds * frames
            GameTime._last_frame = timestamp.frame
            GameTime._platform_timestamp = datetime.datetime.now()
            GameTime._init = True
            GameTime._carla_time = timestamp.elapsed_seconds

    @staticmethod
    def restart():
        """Reset game timer to 0."""
        GameTime._current_game_time = 0.0
        GameTime._carla_time = 0.0
        GameTime._last_frame = 0
        GameTime._init = False

    @staticmethod
    def get_time():
        """Return elapsed game time."""
        return GameTime._current_game_time

    @staticmethod
    def get_carla_time():
        """Return elapsed game time."""
        return GameTime._carla_time

    @staticmethod
    def get_wall_clock_time():
        """Return elapsed game time."""
        return GameTime._platform_timestamp

    @staticmethod
    def get_frame():
        """Return elapsed game time."""
        return GameTime._last_frame
