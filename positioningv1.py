import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import serial

# The latitude/longitude of the datum
# NOTE: This datum is where the "scene's" origin is - the earth will be assumed
#       to be *locally* flat.
from pandas._libs import json

datum_lat = 52.544926#52.5719482
datum_lon = 5.920796#5.814618
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


class PositionEstimator:
    def __init__(self):
        # Initial conditions
        # NOTE: [x,y,z,vx,vy,vz]
        self.estimate = np.array([0, 0, 0, 0, 0, 0])
        self.estimate_covariance = np.identity(self.estimate.size)
        np.fill_diagonal(self.estimate_covariance, np.array([10000 ** 2, 10000 ** 2, 10000 ** 2, 50 ** 2, 50 ** 2, 50 ** 2]))

        # Assume accels are accurate to +- 0.5m/s^2 in all axis and uncorrelated between axis
        self.accel_axis_variance = 0.5 ** 2
        self.accel_covariance = self.accel_axis_variance * np.identity(3)

        # Helper variable for creating identity matrices and the kind
        self.state_size = self.estimate.size
        self.half_state_size = int(self.state_size / 2)

    def run_filter_extrapolation(self, dt, accel_measurement):
        """
        Simply extrapolates the filter using acceleration results
        :param dt: time since last extrapolation
        :param accel_measurement: accelerometer measurements in inertial coordinate system (np 1x3 array; NED)
        """

        # Extrapolate the state
        F = np.vstack((np.hstack((np.identity(self.half_state_size), dt * np.identity(self.half_state_size))),
                       np.hstack((np.zeros((self.half_state_size, self.half_state_size)),
                                  np.identity(self.half_state_size)))))
        G = dt * np.vstack((0.5*(dt**2)*np.identity(self.half_state_size), dt*np.identity(self.half_state_size)))

        # Estimate covariance - obtained from the gyro's accuracy combined with how gyros affect the model (e.g. larger
        # dt means less certainty in the model)
        # pn = 1000 * dt * np.vstack((np.zeros((self.half_state_size, self.half_state_size)), np.identity(self.half_state_size)))
        Q = G @ self.accel_covariance @ G.T

        #print(F)
        #print(self.estimate)
        #print("===")
        #print(G)
        #print(accel_measurement)
        #print(G.dot(accel_measurement))
        #print(G.shape)
        #print(accel_measurement.shape)
        self.estimate = F @ self.estimate + G.dot(accel_measurement)
        self.estimate_covariance = F @ self.estimate_covariance @ F.T + Q

    def run_filter_measurement(self, gps_measurement_data):
        # Compute Kalman gain
        # NOTE: Since measuremennt is a dict rather than a vector, we're just going to extract the relevant info ->
        # currently (since no bias/other stuff in state) our observation matrix will just be I
        scene_pos = glob_to_scene(gps_measurement_data['lat'], gps_measurement_data['lon'], gps_measurement_data['h'])
        #print(f"pos=>{scene_pos}")
        #print(f"vN=>{gps_measurement_data['vN']};acc={gps_measurement_data['sacc']}")

        in_measurement = np.array([scene_pos[0], scene_pos[1], scene_pos[2], gps_measurement_data['vN'], gps_measurement_data['vE'], gps_measurement_data['vD']])

        H = np.identity(self.state_size)

        R = np.identity(self.state_size)
        R[0, 0] = gps_measurement_data['hacc']**2
        R[1, 1] = gps_measurement_data['hacc']**2
        R[2, 2] = gps_measurement_data['vacc']**2
        R[3, 3] = gps_measurement_data['sacc']**2
        R[4, 4] = gps_measurement_data['sacc']**2
        R[5, 5] = gps_measurement_data['sacc']**2
        #R *= 10
        kalman_gain = self.estimate_covariance @ H.T @ np.linalg.inv(
            H @ self.estimate_covariance @ H.T + R)
        print(f"R={R}")
        print(kalman_gain)

        # Create new estimate & corresponding covariance
        self.estimate = self.estimate + kalman_gain @ (in_measurement - H @ self.estimate)
        self.estimate_covariance = (np.identity(
            self.state_size) - kalman_gain @ H) @ self.estimate_covariance @ (
                                           np.identity(
                                               self.state_size) - kalman_gain @ H).T + kalman_gain @ R @ kalman_gain.T

    def get_estimate(self):
        return self.estimate

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

position_estimator = PositionEstimator()
last_position_extrapolation_time = time.time()

# Start a simple HTTP server so we can query the data from a browser/JS (easier real-time-ish visualization)
class DataRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        estimate = position_estimator.get_estimate()
        lat, lon, height = scene_to_glob(estimate[0], estimate[1], estimate[2])
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({'lat': lat, 'lon': lon, 'height': height}).encode())
        return

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        BaseHTTPRequestHandler.end_headers(self)


def run_server_thread():
    webServer = HTTPServer(('127.0.0.1', 5000), DataRequestHandler)
    webServer.serve_forever()
data_thread = threading.Thread(target=run_server_thread)
data_thread.start()

first_fix = False
while True:
    current_time = time.time()
    if stateFlags & ACCEL_AVAIL_FLAG and stateFlags & GYRO_AVAIL_FLAG:
        attitude_estimator.run_filter(current_time - last_attitude_time, accelState, gyroState)
        last_attitude_time = current_time
        #print(f"update_flag:{stateFlags:#010b}")
        ###print(f"Estimate: {attitude_estimator.get_estimate_ypr()}")
        # Clear the accel+gyro flag (TODO: do this at ennd in case another filter needs this data?)
        #stateFlags &= ~(ACCEL_AVAIL_FLAG | GYRO_AVAIL_FLAG)
        # Temp workaround: only clear gyro since others do not rely on it
        stateFlags &= ~(GYRO_AVAIL_FLAG)

    # Position filter - since GPS updates are pretty infrequent, it will be extrapolated more often than it is compared
    # with the (GPS) measuremennts
    if stateFlags & ACCEL_AVAIL_FLAG:
        # Extrapolate position estimate
        # Compute timestep
        dt = current_time - last_position_extrapolation_time
        last_position_extrapolation_time = current_time

        # Convert accelerometer data into more useful acceleration vector (with gravity removed - gravity is assumed to
        # cause a 1g acceleration) in WORLD/SCENE, not body coordinates
        #yaw = np.deg2rad(attitude_estimator.get_estimate_ypr()['pitch'])#0#np.deg2rad(attitude_estimator.get_estimate_ypr()['yaw'])
        #pitch = -np.deg2rad(attitude_estimator.get_estimate_ypr()['yaw'])#0#np.deg2rad(attitude_estimator.get_estimate_ypr()['pitch'])
        #roll = attitude_estimator.get_estimate_ypr()['roll']
        pitch = -np.deg2rad(attitude_estimator.get_estimate_ypr()['yaw'])
        yaw = np.deg2rad(attitude_estimator.get_estimate_ypr()['pitch'])
        roll = attitude_estimator.get_estimate_ypr()['roll']
        body_to_inertial_matrix = np.array([
            [np.cos(roll)*np.cos(pitch), np.cos(pitch)*np.sin(roll), -np.sin(pitch)],
            [np.cos(roll)*np.sin(yaw)*np.sin(pitch) - np.cos(yaw)*np.sin(roll), np.cos(yaw)*np.cos(roll) + np.sin(yaw)*np.sin(roll)*np.sin(pitch), np.cos(pitch)*np.sin(yaw)],
            [np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(roll)*np.sin(pitch), np.cos(yaw)*np.sin(roll)*np.sin(pitch)-np.cos(roll)*np.sin(yaw), np.cos(yaw)*np.cos(pitch)]])
        inertial_to_body_matrix = body_to_inertial_matrix.T  # Assume orthogonal matrix -> inverse is transpose
        # Convert accelerometer measurement to inertial frame + subtract gravity (assume 1g)
        in_accel = np.array([accelState['x'], accelState['y'], accelState['z']])
        accel_vector_inertial = (body_to_inertial_matrix.T @ in_accel - np.array([0, 0, 1.1]))*9.81  # Apparently gravity is 1.1???
        #print(f"{in_accel} -> {accel_vector_inertial}")

        position_estimator.run_filter_extrapolation(dt, accel_vector_inertial) #, np.array([0, 0, 0]))
        stateFlags &= ~ACCEL_AVAIL_FLAG

    if stateFlags & GPS_AVAIL_FLAG:
        # Ensure minimum accuracy requirements are met
        stateCopy = gpsState.copy()
        last_estimate = position_estimator.get_estimate()
        if stateCopy['hacc'] < 15 and stateCopy['vacc'] < 15 and stateCopy['sacc'] < 1:
            if True:#not first_fix:
                position_estimator.run_filter_measurement(stateCopy)
                first_fix = True
        else:
            print(f"WARNING: Minimum GPS accuracy requirements not met! (RAW: {gpsState})")

        stateFlags &= ~GPS_AVAIL_FLAG
        estimate = position_estimator.get_estimate()

        delta = estimate - last_estimate
        if delta[0] > 1 or delta[1] > 1 or delta[2] > 1:
            print("JUMP")
            print(delta)
            print(stateCopy)
        #print(scene_to_glob(estimate[0], estimate[1], estimate[2]))
        #print(position_estimator.get_estimate_covariance())

    time.sleep(0.05)
    continue
    if measurement:
        print(glob_to_scene(measurement['lat'], measurement['lon'], 0))
    else:
        print("NO FIX!")
