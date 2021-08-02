import numpy as np
import serial
import time
import scipy.linalg

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
estimate = np.array([0, 0, 0, 0, 0, 0])
estimate_covariance = np.identity(estimate.size)
np.fill_diagonal(estimate_covariance, np.array([60**2, 60**2, 60**2, 5**2, 5**2, 5**2]))

# Assume gyros are accurate to +- 1deg/s in all axis and uncorrelated between axis
gyro_axis_variance = 1**2
gyro_covariance = gyro_axis_variance * np.identity(3)

# Assume the yaw, pitch and roll signals are accurate to +- 5 deg
accel_converted_variance = 1**2
accel_converted_covariance = accel_converted_variance * np.identity(3)

# Helper variable for creating identity matrices and the kind
state_size = estimate.size
half_state_size = int(state_size/2)

# Store yaw rates as global so we can run the extrapolation knowing the "real" dt + don't use future yaw rates on the old state
u = np.array([0, 0, 0])

time_dps = []
measurement_dps = []
estimate_dps = []

# Ensure we only print some, not all results
counter = 0

# Run "forever", but allow ctrl+C to trigger closing procedures
try:
    last_t = time.time()#time_ns()
    while True and len(time_dps) < 1000:
        # Capture a measurement
        measurement = get_measurement()

        # Compute the timestep since the last measurement in seconds
        cur_t = time.time()
        dt = (cur_t - last_t)
        last_t = cur_t

        print(dt)

        # Extrapolate the state
        F = np.vstack((np.hstack((np.identity(half_state_size), -dt*np.identity(half_state_size))), np.hstack((np.zeros((half_state_size, half_state_size)), np.identity(half_state_size)))))
        G = dt * np.vstack((np.identity(half_state_size), np.zeros((half_state_size, half_state_size))))



        # Estimate covariance - obtained from the gyro's accuracy combined with how gyros affect the model (e.g. larger
        # dt means less certainty in the model)
        #pn = 1000 * dt * np.vstack((np.zeros((half_state_size, half_state_size)), np.identity(half_state_size)))
        Q = G @ gyro_covariance @ G.T# + 0.2**2 * np.identity(state_size)
        #Q[3:, 3:] = gyro_covariance
        print(Q)
        print(F)

        extrapolated_estimate = F @ estimate + G @ u
        extrapolated_estimate_covariance = F @ estimate_covariance @ F.T + Q

        # Compute Kalman gain
        # NOTE: Since measuremennt is a dict rather than a vector, we're just going to extract the relevant info ->
        # currently (since no bias/other stuff in state) our observation matrix will just be I
        in_measurement = np.array([measurement['yaw'], measurement['pitch'], measurement['roll']])

        H = np.vstack((np.identity(half_state_size), np.zeros((half_state_size, half_state_size)))).T

        R = accel_converted_covariance
        kalman_gain = extrapolated_estimate_covariance @ H.T @ np.linalg.inv(H @ extrapolated_estimate_covariance @ H.T + R)

        # Create new estimate & corresponding covariance
        estimate = extrapolated_estimate + kalman_gain @ (in_measurement - H @ extrapolated_estimate)
        estimate_covariance = (np.identity(state_size) - kalman_gain @ H) @ extrapolated_estimate_covariance @ (np.identity(state_size) - kalman_gain @ H).T + kalman_gain @ R @ kalman_gain.T

        # Create the input variable from the measurement => [wx, wy, wz]
        u = np.array([measurement['gz'], measurement['gx'], measurement['gy']])

        time_dps.append(cur_t)
        measurement_dps.append(in_measurement)
        estimate_dps.append(estimate)

        print(estimate[1])

except (KeyboardInterrupt, IndexError, KeyError):
    # Was just KeyboardInterrupt first, but that could cause issues in called fn
    pass

print("plotting...")
plt.scatter(time_dps, np.array(measurement_dps)[:, 1])
plt.plot(time_dps, np.array(estimate_dps)[:, 1], color='orange')
#plt.plot(time_dps, np.array(estimate_dps)[:, 0], label="yaw")
#plt.plot(time_dps, np.array(estimate_dps)[:, 1], label="pitch")
#plt.plot(time_dps, np.array(estimate_dps)[:, 2], label="roll")
plt.xlabel("time")
plt.ylabel("val")
plt.show()

# Close the serial connection
ser.close()

print("done")
