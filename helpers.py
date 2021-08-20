import asyncio
import serial_asyncio


async def establish_device_connection(use_serial=False):
    if use_serial:
        reader, writer = await serial_asyncio.open_serial_connection(url='/dev/tty.usbserial-A900HIOM', baudrate=2000000)
        return reader, writer

    # Establish socket connnection to ESP8266 that outputs incoming serial data to a socket
    reader, writer = await asyncio.open_connection('ESP-0F1694.home', 5000)
    # NOTE: Also returning the writer to ensure we can close the connnection manually if needed
    return reader, writer


async def get_measurement(ser):
    try:
        line = ""
        while line == "":
            try:
                line = (await ser.readline()).decode().strip('\x00')
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