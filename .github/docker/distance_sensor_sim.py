"""
This script will mock the distance sensor via MAVLink messages
For use with simulator, should not be used for real flight
"""
import os
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "common"
import sys
import time
import math
import requests
from pymavlink import mavutil  # type: ignore
from pymavlink.dialects.v20 import common as mavlink
from pymavlink.dialects.v20.common import MAVLink_altitude_message

CONNECTION_URI = "tcp:127.0.0.1:5760"
SYS_ID = int(os.getenv("SYSTEM_ID", 1))

if __name__ == "__main__":
    print("Connecting to drone")
    drone_connection = mavutil.mavlink_connection(
        CONNECTION_URI,
        source_system=SYS_ID,
        source_component=42
    )

    drone_connection.wait_heartbeat()
    print("Connected to drone")

    should_publish_distance = mavutil.periodic_event(5)  # Hz
    should_get_ground_level = mavutil.periodic_event(0.1)  # Once every 10 second should be good enough

    # Default location FAKTRY until we get better data
    drone_lat = 63.3970
    drone_lon = 10.4018
    drone_alt_amsl = 44.2
    drone_home_amsl = math.nan
    ground_level = math.nan
    while True:
        drone_connection.select(0.1)
        msg = drone_connection.recv_match(type=['GLOBAL_POSITION_INT', 'HOME_POSITION'])
        if msg is not None:
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                drone_lat = msg.lat / 1e7
                drone_lon = msg.lon / 1e7
                drone_alt_amsl = msg.alt / 1e3  # m float
            elif math.isnan(drone_home_amsl) and msg.get_type() == 'HOME_POSITION':
                drone_home_amsl = msg.altitude / 1e3

        if should_publish_distance.trigger():
            alt = 0  # cm AGL, integer
            if not math.isnan(ground_level):
                alt = max(0, int((drone_alt_amsl - ground_level) * 100))
            elif not math.isnan(drone_home_amsl):
                alt = max(0, int((drone_alt_amsl - drone_home_amsl) * 100))
            drone_connection.mav.distance_sensor_send(time_boot_ms=0,
                                                      min_distance=10,
                                                      max_distance=20000,
                                                      current_distance=alt,
                                                      type=mavlink.MAV_DISTANCE_SENSOR_LASER,
                                                      id=42,
                                                      orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
                                                      covariance=0
                                                      )
        if should_get_ground_level.trigger():
            try:
                r = requests.get(f'https://ws.geonorge.no/hoydedata/v1/punkt?koordsys=4326&nord={drone_lat}&ost={drone_lon}')
                r.raise_for_status()
                j = r.json()
                tmp = j['punkter'][0]['z']
                if math.isfinite(tmp):
                    ground_level = tmp
                print(ground_level)
            except Exception as e:
                print(e)



