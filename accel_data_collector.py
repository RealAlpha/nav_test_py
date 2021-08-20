import asyncio
import numpy as np
import time
import pandas
from helpers import establish_device_connection, get_measurement

data_buffer = []
wants_stop = False  # Easy way to stop coroutinne


# NOTE: Will call some blocking stuff but the easiest way to call other async stuff/to read data
async def collect_samples():
    print("Establishing connection...")
    reader, writer = await establish_device_connection(use_serial=False)
    print("Connected!")
    while not wants_stop and len(data_buffer) < 1E6:
        measurement = await get_measurement(reader)
        # NOTE: Checking number dtypes to avoid half packets triggering a measurement but then containing trash data with
        # multiple dots/only half the buffer due to printing somethingn from another packet mid-packet
        if measurement is not None and measurement.get('dev') == 'ACCEL' and (
            type(measurement.get('x')) == float and type(measurement.get('y')) == float and type(measurement.get('z')) == float
        ):
            measurement['t'] = time.time()
            data_buffer.append(measurement)

    print("Stopping (1M samples connected or exit requested...")


if __name__ == '__main__':
    try:
        asyncio.get_event_loop().create_task(collect_samples())
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        wants_stop = True
    finally:
        print("Saving data....")
        df = pandas.DataFrame(data_buffer, columns=['t', 'x', 'y', 'z'])
        df.to_feather('data_dump_3.bin')
        print("Done!")
