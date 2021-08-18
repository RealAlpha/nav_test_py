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
    reader, writer = await establish_device_connection()
    print("Connected!")
    while not wants_stop and len(data_buffer) < 1E6:
        measurement = await get_measurement(reader)
        if measurement is not None and measurement.get('dev') == 'ACCEL':
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
        df = pandas.DataFrame(data_buffer, columns=['x', 'y', 'z'])
        df.to_feather('data_dump_3.bin')
        print("Done!")
