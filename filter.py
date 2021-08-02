import numpy as np
import scipy
from numpy.random import default_rng
import matplotlib.pyplot as plt


def get_true_value(t):
    # TODO
    # return np.sin(t)*np.array([1, 1])
    return np.array([.5*t**2, .5*t**2])


rng = default_rng()
measurement_stdev = 3


def get_measurement(t):
    return rng.normal(get_true_value(t), measurement_stdev), np.array([[measurement_stdev**2, 0], [0, measurement_stdev**2]])


# Acceleration stdev
sigma_a = 0.15

# Initial state & variances
state_size = 6
estimate = np.zeros(state_size)
variance = np.zeros((state_size, state_size))
np.fill_diagonal(variance, 500)
#variance[0,0] = 500
#variance[2,2] = 500

dt = 0.1

t_vals = []
true_vals = []
measurements = []
estimations = []
errors = []

for t in np.arange(0, 20+dt, dt):
    # One of the sections (same for both x and y - allows us to create the 6x6 matrix more easily without typing the entire thing out
    kinematic_section = np.array([[1, dt, .5*dt**2], [0, 1, dt], [0, 0, 1]])

    # State extrapolation matrix
    F = np.vstack((np.hstack((kinematic_section.copy(), np.zeros((3, 3)))), np.hstack((np.zeros((3, 3)), kinematic_section.copy()))))

    # NOTE: Update matrix does not exist // G (or \vec{u} for that matter) is not used

    # Process noise (-> arising from acceleration not really being const)
    noise_section = np.array([[0,0,0],[0,0,0],[0,0,1]])
    Q = F @ np.vstack((np.hstack((noise_section.copy(), np.zeros((3, 3)))), np.hstack((np.zeros((3, 3)), noise_section.copy()))))*sigma_a**2 @ F.T

    # Observation matrix
    H = np.array([[1, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0]])

    measurement, measurement_variance = get_measurement(t)

    # Perform model extrapolation
    # TODO: Technically should be doing this on the last iteration...maybe?
    #       it shouldn't matter for now, since F and Q are const, but still...
    model_estimate = F @ estimate
    model_covariance = F @ variance @ F.T + Q

    # Compute the kalman gain
    #print(model_covariance @ H.T)
    kalman_gain = (model_covariance @ H.T) @ np.linalg.inv(H @ model_covariance @ H.T + measurement_variance)

    # Compute new estimate/variance from merging the extrapolation and measurement
    estimate = model_estimate + kalman_gain @ (measurement - H @ model_estimate)
    variance = (np.identity(state_size) - kalman_gain @ H) @ model_covariance @ np.transpose(np.identity(state_size) - kalman_gain @ H) + kalman_gain @ measurement_variance @ kalman_gain.T

    # Collect values for plotting
    t_vals.append(t)
    true_vals.append(get_true_value(t))
    measurements.append(measurement)
    estimations.append(estimate)
    # Position error in at most 99.7% of cases (so three stdevs)
    errors.append([3*np.sqrt(variance[2, 2]), 3*np.sqrt(variance[5,5])])

    #print(f"comp1={model_covariance @ H.T}")
    #print(f"comp2={np.linalg.inv(H@model_covariance@H.T + measurement_variance)}")

    print(f"model={model_covariance}; kalman={kalman_gain}; F={F}; Q={Q}; R={measurement_variance}; estimate={estimate}; var={variance}")

# Plot x vals
plt.scatter(t_vals, np.array(measurements)[:,0], label='measurements', color='blue')
plt.plot(t_vals, np.array(true_vals)[:,0], label='true', color='orange')
plt.errorbar(t_vals, np.array(estimations)[:,0], np.array(errors)[:,0], label='estimate (99.7%)', color='green')
plt.legend()
plt.show()

# Plot some (VERY) basic stats about the track's accuracy
# NOTE: Not excluding the first few things where it totally hasn't converged yet
print(f"Filter R^2={np.average(((np.array(estimations)[:,0] - np.array(true_vals)[:, 0])**2))}; measure R^2={np.average(((np.array(measurements)[:,0] - np.array(true_vals)[:, 0])**2))}")

