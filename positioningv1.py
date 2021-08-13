import threading
import time

import numpy as np
import serial

# The latitude/longitude of the datum (Enter, the Netherlands)
# NOTE: This datum is where the "scene's" origin is - the earth will be assumed
#       to be *locally* flat.
datum_lat = 52.2948
datum_lon = 6.5781
datum_height = 0

# Distance from the origin / used to convert degs -> positions. TODO: Pull data from WGS, this just assumes spherical
#  earth
datum_radius = 6371000


def glob_to_scene(lat, lon, height):
    """
    Takes in geodetic coordinates (relative to the Earth) and converts them into (x,y,z) scene coordinates

    :param lat: Geodetic latitude in deg
    :param lon: Geodetic longitude in deg
    :param height: Height relative to WGS84 ellipsoid in m
    :return: Tuple containing (x,y,z) scene coordinates relative to the datum
    """
    # NOTE: Could, in theory, use smol angle approx
    return np.sin(np.deg2rad(lat-datum_lat))*6371000, np.sin(np.deg2rad(lon-datum_lon))*6371000, height-datum_height


def scene_to_glob(x, y, z):
    """
    Takes in scene coordinates (relative to the datum) and converts them into geodetic coordinates

    :param x: Scene X position
    :param y: Scene Y position
    :param z: Scene Z position
    :return: Tuple containing (lat,lon,height) gedetic coordinates relative to the Earth in deg/m
    """
    return np.rad2deg(np.arcsin(x / datum_radius)) + datum_lat, np.rad2deg(np.arcsin(y / datum_radius)) + datum_lon, z + datum_height


#print(scene_to_glob(*glob_to_scene(10.000000000001,10,100)))



def get_measurement(ser):
    try:
        line = ""
        while line == "":
            try:
                line = ser.readline().decode().strip('\x00')
            except:
                # Retry
                continue

        pairs = line.split(",")
        parse_result = {}
        for pair in pairs:
            # Attempt to convert the value to a float, but leave it as a string when this is impossible (e.g. for 'dev')
            value = pair.split(":")[1]
            try:
                value = float(value)
            except ValueError:
                pass
            parse_result[pair.split(":")[0]] = value

        return parse_result
    except BaseException as e:
        print(e)
        # Handles random exceptions and "failure" messages like: "NO FIX!"
        return None


ACCEL_AVAIL_FLAG = 0b1
GYRO_AVAIL_FLAG = 0b1 << 1
GPS_AVAIL_FLAG = 0b1 << 2

# Various globals that allow the receiving thread to tell the mainn program about updates (much like the micro will
# eventually work with its interrupts)
stateFlags = 0
accelState = {}
gyroState = {}
gpsState = {}


def perform_data_acquisition():
    """
    Helper function that runs an infinite loop updatinng the staet based on serial data.
    NOTE: Should be run in a thread.
    """
    # Esure we ca access the globals from this thread/functionn
    global stateFlags
    global accelState
    global gyroState
    global gpsState

    # Establish a serial connection
    ser = serial.Serial('/dev/tty.usbserial-A900HIOM', 1000000)

    # Perform data acquisition until some form of exception is raised/the thread is otherwise terminated
    while True:
        measurement = get_measurement(ser)
        if not measurement:
            continue

        dev = measurement.get('dev')
        if dev == 'GYRO':
            stateFlags |= GYRO_AVAIL_FLAG
            gyroState = measurement
        elif dev == 'ACCEL':
            stateFlags |= ACCEL_AVAIL_FLAG
            accelState = measurement
        elif dev == 'GPS':
            stateFlags |= GPS_AVAIL_FLAG
            gpsState = measurement


# Start the data acquisition thread
data_thread = threading.Thread(target=perform_data_acquisition)
data_thread.start()

while True:
    print(f"update_flag:{stateFlags:#010b}")
    stateFlags = 0
    time.sleep(0.05)
    continue
    if measurement:
        print(glob_to_scene(measurement['lat'], measurement['lon'], 0))
    else:
        print("NO FIX!")
