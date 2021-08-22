# Simple data logger/storage system
import asyncio
import time

import pandas

from helpers import DataLink, gps_data_packet, accel_data_packet, mag_data_packet, gyro_data_packet

wants_exit = False

async def collect_data():
    link = DataLink(packet_filter=[gps_data_packet, accel_data_packet, mag_data_packet, gyro_data_packet], raw_mode=False)
    await link.connect()

    gps_packets = []
    accel_packets = []
    mag_packets = []
    gyro_packets = []

    try:
        while not wants_exit:
            measurement = await link.get_measurement()
            t_collected = time.time()  # TODO: Since certain things might be batched together, we might want to send the time of transmission as well? // use HAL_GetTicks() [possibly in the protocol/header itself]
            # Since column names and stuff don't match between everything, we need to store them in seperate dataframes
            if type(measurement).__name__ == gps_data_packet.named_tuple.__name__:
                gps_packets.append({'t': t_collected, **measurement._asdict()})
            elif type(measurement).__name__ == accel_data_packet.named_tuple.__name__:
                accel_packets.append({'t': t_collected, **measurement._asdict()})
            elif type(measurement).__name__ == mag_data_packet.named_tuple.__name__:
                mag_packets.append({'t': t_collected, **measurement._asdict()})
            elif type(measurement).__name__ == gyro_data_packet.named_tuple.__name__:
                gyro_packets.append({'t': t_collected, **measurement._asdict()})
            else:
                print("Packet not matched!")

    except:
        pass

    print("Exit requested or exception encountered! Saving...")

    # Store all collected data
    gps_df = pandas.DataFrame(gps_packets)
    gps_df.to_parquet('dump/gps.bin')
    accel_df = pandas.DataFrame(accel_packets)
    accel_df.to_parquet('dump/accel.bin')
    mag_df = pandas.DataFrame(mag_packets)
    mag_df.to_parquet('dump/mag.bin')
    gyro_df = pandas.DataFrame(gyro_packets)
    gyro_df.to_parquet('dump/gyro.bin')

    #print(gps_df.describe())
    #print(accel_df.describe())
    #print(mag_df.describe())
    #print(gyro_df.describe())

    print("Finished saving!")


if __name__ == '__main__':
    asyncio.get_event_loop().create_task(collect_data())
    try:
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        # Attempt to stop the main body
        wants_exit = True
        asyncio.get_event_loop().run_forever()
