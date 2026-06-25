#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Wrapper for autonomous agents required for tracking and checking of used sensors
"""

from __future__ import print_function
import math
import os
import time

import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime

from leaderboard.envs.sensor_interface import CallBack, OpenDriveMapReader, SpeedometerReader, SensorConfigurationInvalid
from leaderboard.autoagents.autonomous_agent import Track
from leaderboard.autoagents.ros_base_agent import ROSBaseAgent

IS_BENCH2DRIVE = os.environ.get('IS_BENCH2DRIVE', None)
if IS_BENCH2DRIVE:
    MAX_ALLOWED_RADIUS_SENSOR = 100.0  # for visualize
else:
    MAX_ALLOWED_RADIUS_SENSOR = 3.0

QUALIFIER_SENSORS_LIMITS = {
    'sensor.camera.rgb': 4,
    'sensor.lidar.ray_cast': 1,
    'sensor.other.radar': 2,
    'sensor.other.gnss': 1,
    'sensor.other.imu': 1,
    'sensor.opendrive_map': 1,
    'sensor.speedometer': 1
}
SENSORS_LIMITS = {
    'sensor.camera.rgb': 8,
    'sensor.lidar.ray_cast': 2,
    'sensor.other.radar': 4,
    'sensor.other.gnss': 1,
    'sensor.other.imu': 1,
    'sensor.opendrive_map': 1,
    'sensor.speedometer': 1
}
ALLOWED_SENSORS = SENSORS_LIMITS.keys()


class AgentError(Exception):
    """
    Exceptions thrown when the agent returns an error during the simulation
    """

    def __init__(self, message):
        super(AgentError, self).__init__(message)

        
class TickRuntimeError(Exception):
    pass

class AgentWrapperFactory(object):

    @staticmethod
    def get_wrapper(agent):
        if isinstance(agent, ROSBaseAgent):
            return ROSAgentWrapper(agent)
        else:
            return AgentWrapper(agent)


def validate_sensor_configuration(sensors, agent_track, selected_track):
    """
    Ensure that the sensor configuration is valid, in case the challenge mode is used
    Returns true on valid configuration, false otherwise
    """
    if Track(selected_track) != agent_track:
        raise SensorConfigurationInvalid("You are submitting to the wrong track [{}]!".format(Track(selected_track)))

    sensor_count = {}
    sensor_ids = []

    for sensor in sensors:

        # Check if the is has been already used
        sensor_id = sensor['id']
        if sensor_id in sensor_ids:
            raise SensorConfigurationInvalid("Duplicated sensor tag [{}]".format(sensor_id))
        else:
            sensor_ids.append(sensor_id)

        # Check if the sensor is valid
        if agent_track == Track.SENSORS:
            if sensor['type'].startswith('sensor.opendrive_map'):
                raise SensorConfigurationInvalid("Illegal sensor 'sensor.opendrive_map' used for Track [{}]!".format(agent_track))

        # Check the sensors validity
        if sensor['type'] not in ALLOWED_SENSORS:
            raise SensorConfigurationInvalid("Illegal sensor '{}' used for Track [{}]!".format(sensor['type'], agent_track))

        # Check the extrinsics of the sensor
        if 'x' in sensor and 'y' in sensor and 'z' in sensor:
            if math.sqrt(sensor['x']**2 + sensor['y']**2 + sensor['z']**2) > MAX_ALLOWED_RADIUS_SENSOR:
                raise SensorConfigurationInvalid(
                    "Illegal sensor extrinsics used for sensor '{}'. Max allowed radius is {}m".format(sensor['id'], MAX_ALLOWED_RADIUS_SENSOR))

        # Check the amount of sensors
        if sensor['type'] in sensor_count:
            sensor_count[sensor['type']] += 1
        else:
            sensor_count[sensor['type']] = 1

    if agent_track in (Track.SENSORS_QUALIFIER, Track.MAP_QUALIFIER):
        sensor_limits = QUALIFIER_SENSORS_LIMITS
    else:
        sensor_limits = SENSORS_LIMITS

    for sensor_type, max_instances_allowed in sensor_limits.items():
        if sensor_type in sensor_count and sensor_count[sensor_type] > max_instances_allowed:
            raise SensorConfigurationInvalid(
                "Too many {} used! "
                "Maximum number allowed is {}, but {} were requested.".format(sensor_type,
                                                                              max_instances_allowed,
                                                                              sensor_count[sensor_type]))


class AgentWrapper(object):

    """
    Wrapper for autonomous agents required for tracking and checking of used sensors
    """
    _pooled_sensor_ids = set()
    _sensor_pool = {}

    def __init__(self, agent):
        """
        Set the autonomous agent
        """
        self._agent = agent
        self._sensors_list = []

    def __call__(self):
        """
        Pass the call directly to the agent
        """
        return self._agent()

    def _preprocess_sensor_spec(self, sensor_spec):
        type_ = sensor_spec["type"]
        id_ = sensor_spec["id"]
        attributes = {}

        if type_ == 'sensor.opendrive_map':
            attributes['reading_frequency'] = sensor_spec['reading_frequency']
            sensor_location = carla.Location()
            sensor_rotation = carla.Rotation()

        elif type_ == 'sensor.speedometer':
            delta_time = CarlaDataProvider.get_world().get_settings().fixed_delta_seconds
            attributes['reading_frequency'] = 1 / delta_time
            sensor_location = carla.Location()
            sensor_rotation = carla.Rotation()

        if type_ == 'sensor.camera.rgb':
            attributes['image_size_x'] = str(sensor_spec['width'])
            attributes['image_size_y'] = str(sensor_spec['height'])
            attributes['fov'] = str(sensor_spec['fov'])
            self._copy_optional_sensor_attributes(
                sensor_spec,
                attributes,
                ['sensor_tick', 'enable_postprocess_effects', 'motion_blur_intensity']
            )

            sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                             roll=sensor_spec['roll'],
                                             yaw=sensor_spec['yaw'])

        elif type_ == 'sensor.lidar.ray_cast':
            attributes['range'] = str(85)
            attributes['rotation_frequency'] = str(10)
            attributes['channels'] = str(64)
            attributes['upper_fov'] = str(10)
            attributes['lower_fov'] = str(-30)
            attributes['points_per_second'] = str(600000)
            attributes['atmosphere_attenuation_rate'] = str(0.004)
            attributes['dropoff_general_rate'] = str(0.45)
            attributes['dropoff_intensity_limit'] = str(0.8)
            attributes['dropoff_zero_intensity'] = str(0.4)
            self._copy_optional_sensor_attributes(
                sensor_spec,
                attributes,
                [
                    'sensor_tick',
                    'range',
                    'rotation_frequency',
                    'channels',
                    'upper_fov',
                    'lower_fov',
                    'points_per_second',
                    'atmosphere_attenuation_rate',
                    'dropoff_general_rate',
                    'dropoff_intensity_limit',
                    'dropoff_zero_intensity',
                ]
            )

            sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                             roll=sensor_spec['roll'],
                                             yaw=sensor_spec['yaw'])

        elif type_ == 'sensor.other.radar':
            attributes['horizontal_fov'] = str(sensor_spec['horizontal_fov'])  # degrees
            attributes['vertical_fov'] = str(sensor_spec['vertical_fov'])  # degrees
            attributes['points_per_second'] = '1500'
            attributes['range'] = '100'  # meters
            self._copy_optional_sensor_attributes(
                sensor_spec,
                attributes,
                ['sensor_tick', 'points_per_second', 'range']
            )

            sensor_location = carla.Location(x=sensor_spec['x'],
                                             y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                             roll=sensor_spec['roll'],
                                             yaw=sensor_spec['yaw'])

        elif type_ == 'sensor.other.gnss':
            attributes['noise_alt_stddev'] = str(0.000005)
            attributes['noise_lat_stddev'] = str(0.000005)
            attributes['noise_lon_stddev'] = str(0.000005)
            attributes['noise_alt_bias'] = str(0.0)
            attributes['noise_lat_bias'] = str(0.0)
            attributes['noise_lon_bias'] = str(0.0)

            sensor_location = carla.Location(x=sensor_spec['x'],
                                             y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation()

        elif type_ == 'sensor.other.imu':
            attributes['noise_accel_stddev_x'] = str(0.001)
            attributes['noise_accel_stddev_y'] = str(0.001)
            attributes['noise_accel_stddev_z'] = str(0.015)
            attributes['noise_gyro_stddev_x'] = str(0.001)
            attributes['noise_gyro_stddev_y'] = str(0.001)
            attributes['noise_gyro_stddev_z'] = str(0.001)

            sensor_location = carla.Location(x=sensor_spec['x'],
                                             y=sensor_spec['y'],
                                             z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                             roll=sensor_spec['roll'],
                                             yaw=sensor_spec['yaw'])
        sensor_transform = carla.Transform(sensor_location, sensor_rotation)

        return type_, id_, sensor_transform, attributes

    @staticmethod
    def _copy_optional_sensor_attributes(sensor_spec, attributes, names):
        for name in names:
            if name in sensor_spec:
                value = sensor_spec[name]
                if isinstance(value, bool):
                    value = str(value).lower()
                attributes[name] = str(value)

    def setup_sensors(self, vehicle):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param vehicle: ego vehicle
        :return:
        """
        world = CarlaDataProvider.get_world()
        bp_library = world.get_blueprint_library()
        reuse_lead_rig = self._reuse_lead_rig_enabled()
        reuse_lead_sensors = self._reuse_lead_sensors_enabled()
        sensor_specs = self._agent.sensors()
        pool_key = None
        pooled_sensors = {}
        diagnostics = os.environ.get("B2D_TICK_DIAGNOSTICS") == "1"
        if reuse_lead_rig:
            if reuse_lead_sensors:
                pool_key = self._sensor_pool_key(world, vehicle, sensor_specs)
                pooled_sensors = self._sensor_pool.get(pool_key, {})
            print(
                f"lead_rig_pool setup vehicle_id={vehicle.id} key={pool_key} "
                f"pooled={len(pooled_sensors)} sensor_pool={'on' if reuse_lead_sensors else 'off'}",
                flush=True
            )
        try:
            for sensor_spec in sensor_specs:
                type_, id_, sensor_transform, attributes = self._preprocess_sensor_spec(sensor_spec)
                if diagnostics:
                    print(f"sensor_setup_before id={id_} type={type_}", flush=True)

                # These are the pseudosensors (not spawned)
                if type_ == 'sensor.opendrive_map':
                    sensor = OpenDriveMapReader(vehicle, attributes['reading_frequency'])
                elif type_ == 'sensor.speedometer':
                    sensor = SpeedometerReader(vehicle, attributes['reading_frequency'])

                # These are the sensors spawned on the carla world
                else:
                    sensor = None
                    if reuse_lead_sensors and id_ in pooled_sensors:
                        candidate = pooled_sensors[id_]
                        if (
                            candidate is not None
                            and candidate.is_alive
                            and self._sensor_attached_to_vehicle(candidate, vehicle)
                        ):
                            sensor = candidate
                            print(f"lead_rig_pool reuse sensor id={id_} actor_id={sensor.id}", flush=True)
                        else:
                            self._discard_pooled_sensor(candidate)
                            pooled_sensors.pop(id_, None)
                    if sensor is None:
                        bp = bp_library.find(type_)
                        for key, value in attributes.items():
                            bp.set_attribute(str(key), str(value))
                        sensor = CarlaDataProvider.get_world().spawn_actor(bp, sensor_transform, vehicle)
                        if reuse_lead_sensors:
                            pooled_sensors[id_] = sensor
                            self._pooled_sensor_ids.add(sensor.id)
                            print(f"lead_rig_pool spawn sensor id={id_} actor_id={sensor.id}", flush=True)

                # setup callback
                sensor.listen(CallBack(id_, type_, sensor, self._agent.sensor_interface))
                self._sensors_list.append(sensor)
                if diagnostics:
                    actor_id = getattr(sensor, "id", "pseudo")
                    print(f"sensor_setup_after id={id_} type={type_} actor_id={actor_id}", flush=True)

            if reuse_lead_sensors and pool_key is not None:
                self._sensor_pool[pool_key] = pooled_sensors

            # A few frames are enough to prime callbacks. More warm-up ticks can
            # block while large-map tiles are streaming on Windows CARLA.
            warmup_ticks = max(1, int(os.environ.get("B2D_SENSOR_WARMUP_TICKS", "5")))
            for warmup_tick in range(warmup_ticks):
                if diagnostics:
                    print(
                        f"sensor_warmup_before tick={warmup_tick + 1}/{warmup_ticks}",
                        flush=True,
                    )
                frame = world.tick()
                if diagnostics:
                    print(
                        f"sensor_warmup_after tick={warmup_tick + 1}/{warmup_ticks} frame={frame}",
                        flush=True,
                    )
        except Exception:
            self.cleanup(force_destroy_pooled=True)
            raise

    def cleanup(self, force_destroy_pooled=False):
        """
        Remove and destroy all sensors
        """
        world = CarlaDataProvider.get_world()
        reuse_lead_sensors = self._reuse_lead_sensors_enabled()
        for i, _ in enumerate(self._sensors_list):
            if self._sensors_list[i] is not None:
                try:
                    self._sensors_list[i].stop()
                except RuntimeError:
                    pass

        # Give CARLA one frame to drain pending sensor callbacks before destroying
        # actors. This avoids UE render-thread crashes when camera streams are torn
        # down during route cleanup or interruption.
        if world:
            try:
                world.tick()
            except RuntimeError:
                pass

        for i, _ in enumerate(self._sensors_list):
            if self._sensors_list[i] is not None:
                if (
                    reuse_lead_sensors
                    and not force_destroy_pooled
                    and self._is_pooled_sensor(self._sensors_list[i])
                ):
                    print(
                        f"lead_rig_pool keep sensor actor_id={self._sensors_list[i].id}",
                        flush=True
                    )
                else:
                    self._discard_pooled_sensor(self._sensors_list[i])
                    try:
                        self._sensors_list[i].destroy()
                    except RuntimeError:
                        pass
                self._sensors_list[i] = None
        self._sensors_list = []

        # Tick once to destroy the sensors
        if world:
            try:
                world.tick()
            except RuntimeError:
                pass

    @staticmethod
    def _reuse_lead_rig_enabled():
        return (
            os.environ.get('B2D_REUSE_LEAD_RIG', '').strip().lower() in ('1', 'true', 'yes', 'on')
            and os.environ.get('B2D_AGENT_KIND', '').strip().lower() == 'lead'
        )

    @classmethod
    def _reuse_lead_sensors_enabled(cls):
        return (
            cls._reuse_lead_rig_enabled()
            and os.environ.get('B2D_REUSE_LEAD_SENSORS', '').strip().lower() in ('1', 'true', 'yes', 'on')
        )

    @classmethod
    def _normalize_sensor_value(cls, value):
        if isinstance(value, dict):
            return tuple(
                sorted(
                    (str(key), cls._normalize_sensor_value(val))
                    for key, val in value.items()
                )
            )
        if isinstance(value, (list, tuple)):
            return tuple(cls._normalize_sensor_value(item) for item in value)
        return value

    @classmethod
    def _sensor_signature(cls, sensor_specs):
        signature = []
        for sensor in sensor_specs:
            if sensor.get('type') in ('sensor.opendrive_map', 'sensor.speedometer'):
                continue
            signature.append(tuple(
                sorted(
                    (str(key), cls._normalize_sensor_value(value))
                    for key, value in sensor.items()
                )
            ))
        return tuple(sorted(signature))

    @classmethod
    def _sensor_pool_key(cls, world, vehicle, sensor_specs):
        town = world.get_map().name.split('/')[-1]
        return (town, vehicle.id, cls._sensor_signature(sensor_specs))

    @classmethod
    def _is_pooled_sensor(cls, sensor):
        try:
            actor_id = getattr(sensor, 'id', None)
            return actor_id is not None and actor_id in cls._pooled_sensor_ids
        except RuntimeError:
            return False

    @classmethod
    def is_pooled_sensor_actor_id(cls, actor_id):
        return actor_id in cls._pooled_sensor_ids

    @staticmethod
    def _sensor_attached_to_vehicle(sensor, vehicle):
        try:
            parent = getattr(sensor, 'parent', None)
            return parent is not None and parent.id == vehicle.id
        except RuntimeError:
            return False

    @classmethod
    def _discard_pooled_sensor(cls, sensor):
        if sensor is None:
            return
        try:
            actor_id = getattr(sensor, 'id', None)
        except RuntimeError:
            actor_id = None
        if actor_id is None:
            return

        cls._pooled_sensor_ids.discard(actor_id)
        empty_keys = []
        for pool_key, sensors in cls._sensor_pool.items():
            for sensor_id, pooled_sensor in list(sensors.items()):
                try:
                    pooled_actor_id = getattr(pooled_sensor, 'id', None)
                except RuntimeError:
                    pooled_actor_id = None
                if pooled_actor_id == actor_id:
                    sensors.pop(sensor_id, None)
            if not sensors:
                empty_keys.append(pool_key)
        for pool_key in empty_keys:
            cls._sensor_pool.pop(pool_key, None)


class ROSAgentWrapper(AgentWrapper):

    SENSOR_TYPE_REMAPS = {
        "sensor.opendrive_map": "sensor.pseudo.opendrive_map",
        "sensor.speedometer": "sensor.pseudo.speedometer"
    }

    def __init__(self, agent):
        super(ROSAgentWrapper, self).__init__(agent)

    def _preprocess_sensor_spec(self, sensor_spec):
        type_, id_, sensor_transform, attributes = super(ROSAgentWrapper, self)._preprocess_sensor_spec(sensor_spec)
        new_type = self.SENSOR_TYPE_REMAPS.get(type_, type_)
        return new_type, id_, sensor_transform, attributes

    def setup_sensors(self, vehicle):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param vehicle: ego vehicle
        :return:
        """
        for sensor_spec in self._agent.sensors():
            type_, id_, transform, attributes = self._preprocess_sensor_spec(sensor_spec)
            uid = self._agent.spawn_object(type_, id_, transform, attributes, attach_to=vehicle.id)
            self._sensors_list.append(uid)

        # Tick once to spawn the sensors
        CarlaDataProvider.get_world().tick()

    def cleanup(self):
        for uid in self._sensors_list:
            self._agent.destroy_object(uid)
        self._sensors_list.clear()

        # Tick once to destroy the sensors
        CarlaDataProvider.get_world().tick()
