import asyncio
import numpy as np
from helpers import establish_device_connection, get_measurement

NUM_CALIBRATION_SAMPLES = 1000

# NOTE: Will call some blocking stuff but the easiest way to call other async stuff/to read data
async def main():
    while True:
        user_option = input("Enter the axis you'd like to calibrate (x,y,z) or e if you'd like to exit: ")
        if user_option == 'e':
            break

        if user_option not in ['x', 'y', 'z']:
            print("Invalid axis!")
            continue

        # Give the user some time to position the sensor
        input(f"Press enter to start the calibration procedure (n={NUM_CALIBRATION_SAMPLES}). Please make sure your device is in the proper position before doing so!")

        # Establish a connection: this is done here to avoid weird issues when clearing the backlog of messages between
        # measurements -> this way we can be pretty confident that we have the latest stuff
        print("Establishing connection...")
        reader, writer = await establish_device_connection()
        print("Connnected!")
        # OLD: Clear the backlog of samples (else we'll have old data, probably from when the sensor was in the
        # incorrect pos). NOTE: 2**16 comes from the max size, as else it'll just continue forever since EOF is never reached
        #await reader.read(2**16)

        # Read the requested # of samples into an array
        print(f"Reading {NUM_CALIBRATION_SAMPLES} samples....")
        print("Press ctrl+c to exit program.")
        axis = user_option  # Alias to avoid confusion
        samples = np.zeros(NUM_CALIBRATION_SAMPLES)
        i = 0
        try:
            while i < NUM_CALIBRATION_SAMPLES:
                measurement = await get_measurement(reader)
                if measurement is not None and measurement.get('dev') == 'ACCEL' and measurement.get(axis) is not None:
                    samples[i] = measurement.get(axis)
                    i += 1

                    # Print progress every 10%
                    progress_pop_num_samples = int(NUM_CALIBRATION_SAMPLES / 10)
                    if i % progress_pop_num_samples == 0:
                        print(f"{i / progress_pop_num_samples * 10}% done collecting samples.")
        except KeyboardInterrupt:
            # Allow ctrl+c to stop sample collection
            pass

        writer.close()

        print("Finished collecting samples!")
        print(f"Average≈{np.average(samples)}")
        print(f"Standard Deviation≈{np.std(samples)}")
        # NOTE: Want the absolute value to avoid a bias of approx. 2 in case the sensor is upside down / assume bias is same
        #       in both directions
        print(f"Bias (assume gravity does indeed cause a 1g acceleration): {np.abs(np.average(samples)) - 1}")
        print("Done!")

    print("Exiting...")


if __name__ == '__main__':
    asyncio.get_event_loop().create_task(main())
    asyncio.get_event_loop().run_forever()