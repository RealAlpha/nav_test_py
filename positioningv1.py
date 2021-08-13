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


class AttitudeEstimator:
    def __init__(self):
        # Initial conditions
        # NOTE: [y,p,r,by,bp,br]
        self.estimate = np.array([0, 0, 0, 0, 0, 0])
        self.estimate_covariance = np.identity(self.estimate.size)
        np.fill_diagonal(self.estimate_covariance, np.array([60 ** 2, 60 ** 2, 60 ** 2, 5 ** 2, 5 ** 2, 5 ** 2]))

        # Assume gyros are accurate to +- 1deg/s in all axis and uncorrelated between axis
        self.gyro_axis_variance = 1 ** 2
        self.gyro_covariance = self.gyro_axis_variance * np.identity(3)

        # Assume the yaw, pitch and roll signals are accurate to +- 5 deg
        self.accel_converted_variance = 1 ** 2
        self.accel_converted_covariance = self.accel_converted_variance * np.identity(3)

        # Helper variable for creating identity matrices and the kind
        self.state_size = self.estimate.size
        self.half_state_size = int(self.state_size / 2)

        # Store yaw rates as global so we can run the extrapolation knowing the "real" dt + don't use future yaw rates on the old state
        self.u = np.array([0, 0, 0])
    
    def run_filter(self, dt, accel_measurement, gyro_measurement):
        # Convert the accelerometer measurement into euler angles (in degrees)
        yaw = np.rad2deg(np.arctan2(accel_measurement['x'], accel_measurement['z']))
        pitch = np.rad2deg(np.arctan2(accel_measurement['y'], accel_measurement['z']))
        roll = np.rad2deg(np.arctan2(accel_measurement['x'], accel_measurement['y']))
        #print(f"y={yaw},p={pitch},r={roll}")
        #print(accel_measurement)

        # Extrapolate the state
        F = np.vstack((np.hstack((np.identity(self.half_state_size), -dt * np.identity(self.half_state_size))),
                       np.hstack((np.zeros((self.half_state_size, self.half_state_size)), np.identity(self.half_state_size)))))
        G = dt * np.vstack((np.identity(self.half_state_size), np.zeros((self.half_state_size, self.half_state_size))))

        # Estimate covariance - obtained from the gyro's accuracy combined with how gyros affect the model (e.g. larger
        # dt means less certainty in the model)
        # pn = 1000 * dt * np.vstack((np.zeros((self.half_state_size, self.half_state_size)), np.identity(self.half_state_size)))
        Q = G @ self.gyro_covariance @ G.T

        extrapolated_estimate = F @ self.estimate + G @ self.u
        extrapolated_estimate_covariance = F @ self.estimate_covariance @ F.T + Q

        # Compute Kalman gain
        # NOTE: Since measuremennt is a dict rather than a vector, we're just going to extract the relevant info ->
        # currently (since no bias/other stuff in state) our observation matrix will just be I
        in_measurement = np.array([yaw, pitch, roll])

        H = np.vstack((np.identity(self.half_state_size), np.zeros((self.half_state_size, self.half_state_size)))).T

        R = self.accel_converted_covariance
        kalman_gain = extrapolated_estimate_covariance @ H.T @ np.linalg.inv(
            H @ extrapolated_estimate_covariance @ H.T + R)

        # Create new estimate & corresponding covariance
        self.estimate = extrapolated_estimate + kalman_gain @ (in_measurement - H @ extrapolated_estimate)
        self.estimate_covariance = (np.identity(self.state_size) - kalman_gain @ H) @ extrapolated_estimate_covariance @ (
                np.identity(self.state_size) - kalman_gain @ H).T + kalman_gain @ R @ kalman_gain.T

        # Create the input variable from the measurement => [wx, wy, wz]
        self.u = np.array([gyro_measurement['gz'], gyro_measurement['gx'], gyro_measurement['gy']])

    def get_estimate_ypr(self):
        return {
            'yaw': self.estimate[0],
            'pitch': self.estimate[1],
            'roll': self.estimate[2]
        }

    def get_estimate_covariance(self):
        return self.estimate_covariance


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
        #print(measurement)
        #continue
        if not measurement:
            continue

        dev = measurement.get('dev')
        if dev == 'GYRO':
            gyroState = measurement
            stateFlags |= GYRO_AVAIL_FLAG
        elif dev == 'ACCEL':
            accelState = measurement
            stateFlags |= ACCEL_AVAIL_FLAG
        elif dev == 'GPS':
            gpsState = measurement
            stateFlags |= GPS_AVAIL_FLAG


# Start the data acquisition thread
data_thread = threading.Thread(target=perform_data_acquisition)
data_thread.start()

attitude_estimator = AttitudeEstimator()
last_attitude_time = time.time()

while True:
    current_time = time.time()
    if stateFlags & ACCEL_AVAIL_FLAG and stateFlags & GYRO_AVAIL_FLAG:
        attitude_estimator.run_filter(current_time - last_attitude_time, accelState, gyroState)
        last_attitude_time = current_time
        #print(f"update_flag:{stateFlags:#010b}")
        print(f"Estimate: {attitude_estimator.get_estimate_ypr()}")
        # Clear the accel+gyro flag (TODO: do this at ennd in case another filter needs this data?)
        stateFlags &= ~(ACCEL_AVAIL_FLAG | GYRO_AVAIL_FLAG)
    time.sleep(0.05)
    continue
    if measurement:
        print(glob_to_scene(measurement['lat'], measurement['lon'], 0))
    else:
        print("NO FIX!")
