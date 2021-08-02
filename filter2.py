import numpy as np
import serial
import time

import matplotlib.pyplot as plt

ser = serial.Serial('/dev/tty.usbserial-A900HIOM', 112500)


def get_measurement():
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
        parse_result[pair.split(":")[0]] = float(pair.split(":")[1])

    return parse_result


# Initial conditions
# NOTE: [y,p,r,by,bp,br]
estimate = np.array([0, 0, 0])#, 0, 0, 0])
estimate_covariance = np.array([60**2, 60**2, 60**2])#, 5**2, 5**2, 5**2]

# Assume gyros are accurate to +- 1deg/s in all axis and uncorrelated between axis
gyro_axis_variance = 0.7**2
gyro_covariance = gyro_axis_variance * np.identity(3)

# Assume the yaw, pitch and roll signals are accurate to +- 5 deg
accel_converted_variance = 1**2
accel_converted_covariance = accel_converted_variance * np.identity(3)

# Helper variable for creating identity matrices and the kind
state_size = estimate.size

# Store yaw rates as global so we can run the extrapolation knowing the "real" dt + don't use future yaw rates on the old state
u = np.array([0, 0, 0])

time_dps = []
measurement_dps = []
estimate_dps = []


# Run "forever", but allow ctrl+C to trigger closing procedures
try:
    last_t = time.time()#time_ns()
    while True and len(time_dps) < 1000:
        # Capture a measurement
        measurement = get_measurement()

        # Compute the timestep since the last measurement in seconds
        cur_t = time.time()#time_ns()
        dt = (cur_t - last_t)# * 10e-9
        last_t = cur_t

        # Extrapolate the state
        F = np.identity(state_size)
        G = dt * np.identity(state_size)



        # Estimate covariance - obtained from the gyro's accuracy combined with how gyros affect the model (e.g. larger
        # dt means less certainty in the model)
        Q = G @ gyro_covariance @ G.T

        extrapolated_estimate = F @ estimate + G @ u
        extrapolated_estimate_covariance = F @ estimate_covariance @ F.T + Q

        # Compute Kalman gain
        # NOTE: Since measuremennt is a dict rather than a vector, we're just going to extract the relevant info ->
        # currently (since no bias/other stuff in state) our observation matrix will just be I
        in_measurement = np.array([measurement['yaw'], measurement['pitch'], measurement['roll']])
        H = np.identity(state_size)

        R = accel_converted_covariance
        kalman_gain = extrapolated_estimate_covariance @ H.T @ np.linalg.inv(H @ extrapolated_estimate_covariance @ H.T + R)

        # Create new estimate & corresponding covariance
        estimate = extrapolated_estimate + kalman_gain @ (in_measurement - H @ extrapolated_estimate)
        estimate_covariance = (np.identity(state_size) - kalman_gain @ H) @ extrapolated_estimate_covariance @ (np.identity(state_size) - kalman_gain @ H).T + kalman_gain @ R @ kalman_gain.T

        # Create the input variable from the measurement => [wx, wy, wz]
        u = np.array([measurement['gz'], measurement['gx'], measurement['gy']])

        # Perform some data tracking
        #print(f"pitch estimated: {estimate[1]}, measured: {in_measurement[1]}, dt={dt}")
        time_dps.append(cur_t)
        measurement_dps.append(in_measurement[1])
        estimate_dps.append(estimate[1])

except (KeyboardInterrupt, IndexError, KeyError):
    # Was just KeyboardInterrupt first, but that could cause issues in called fn
    pass

print("plotting...")
plt.scatter(time_dps, measurement_dps)
plt.plot(time_dps, estimate_dps, color='orange')
plt.show()


#print(get_measurement())

# Close the serial connnectionn
ser.close()

print("done")
