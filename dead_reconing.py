"""
This file contains various accelerometer/positioning dead reckoning experiments.
"""

import asyncio
import time

import numpy as np
from helpers import establish_device_connection, get_measurement

# Additionnal bias applied in software / post-receive
#software_bias = -1 * np.array([-0.07603999999999989, 0.05544400000000027, 0.02693599999999985])
#software_bias = -1 * np.array([-0.04603999999999989, 0.05544400000000027, 0.02693599999999985])
#software_bias = np.array([ 0.038572, -0.052668, -0.03192 ])
#software_bias = np.array([0.037772, -0.054268, -0.03156])
#software_bias = np.array([0.006624, 0.03456,  0.129392])
#software_bias = np.array([[0.062056, -0.022904, 0.110872]])
#software_bias = np.array([ 0.0489376, -0.0115192,  0.1464368])
software_bias = np.array([0.066948, -0.021644, 0.10842])
sample_stdev = np.array([0.03087235, 0.03611658, 0.09751711])

# NOTE: Will call some blocking stuff but the easiest way to call other async stuff/to read data
async def main():
    global software_bias
    # Connect to the sensor
    print("Establishing connection...")
    reader, writer = await establish_device_connection()
    print("Done!")

    # Global state variables
    last_t = time.time()
    # Velocity
    v = np.zeros(3)
    # Position
    p = np.zeros(3)

    # Number of samples to use for the rolling average
    ROLLING_AVERAGE_SAMPLE_NUM = 25#3
    last_sample_buffer = np.zeros((ROLLING_AVERAGE_SAMPLE_NUM, 3))
    last_sample_index = 0

    # More bias stuff/histogram?
    a_history = []


    while True:
        measurement = await get_measurement(reader)
        if measurement and measurement.get('dev') == 'ACCEL':
            # Convert the measurement into a numpy array to make it easier to work with
            # Subtract gravity (assume sensor points up we receive NED data
            a = np.array([measurement.get('x'), measurement.get('y'), measurement.get('z')]) + np.array([0, 0, 1]) - software_bias
            a_history.append(a)

            # Compute rolling average
            #last_sample_buffer[last_sample_index] = a
            #last_sample_index = (last_sample_index + 1) % ROLLING_AVERAGE_SAMPLE_NUM
            #avg_a = np.average(last_sample_buffer, 0)

            # Ignore all samples that fall within 3 sigma's of 0 // horrible first attempt at filtering out some (more) noise
            avg_a = a
            # if np.all(avg_a <= 2*sample_stdev):
            #     #print("stdev")
            #     continue
            for i in range(len(avg_a)):
                if avg_a[i] < 3*sample_stdev[i]:
                    avg_a[i] = 0

            #print(avg_a)
            # Determine the time step since last measurement
            cur_t = time.time()
            dt = cur_t - last_t
            last_t = cur_t

            #print(a)
            #print(f"a={a}; avg_a={avg_a}")

            p += v * dt + avg_a * 0.5 * dt**2
            v += avg_a * dt

            if len(a_history) % 10 == 0:
                print(f"x={p[0]}; vx={v[0]}")

            #print(f"x={p[0]}; vx={v[0]}")
            if len(a_history) % 10000 == 0:
                avg = np.average(a_history, 0)
                print(f"Average (last 1000 samples): {avg}; norm={np.linalg.norm(avg)}; stdev={np.std(np.array(a_history), axis=0)}")
                software_bias += avg
                print(f"New software bias: {software_bias}")

                # Reset for next run (with new bias - else that'd innvalidate the results or we'd need to transform stuff
                a_history.clear()
        else:
            pass
            #await asyncio.sleep(0.001)




if __name__ == '__main__':
    asyncio.get_event_loop().create_task(main())
    asyncio.get_event_loop().run_forever()
