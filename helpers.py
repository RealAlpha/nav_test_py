import asyncio
import serial_asyncio

from collections import namedtuple
import struct
import binascii


# START OF V1 CONNECTION/MEASUREMENT
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
# END OF V1 CONNECTION/MEASUREMENT


# START OF V2 CONNECTION/MEASUREMENT
class DataPacket:
    """
    Helper class for storing information about the various data sources / how to decode them.
    Used for packet filtering/easier access to the variables
    """
    def __init__(self, pkt_id, fmt, nt_fields, name_override=None):
        """
        Creates a new packet specification that can be registered with a data link.

        :param fmt: The struct packing format string (less any endianness/encoding specifiers - those are added by the
        data link)
        :param nt_fields: A space-delimited string of fields passed to the named tuple
        :param name_override: Optional override to change the namedtuple's name
        """
        self.pkt_id = pkt_id
        self.fmt = fmt
        self.named_tuple = namedtuple(name_override if name_override else DataPacket.__name__, nt_fields)

    def get_id(self):
        return self.pkt_id

    def get_size(self):
        return struct.calcsize(self.fmt)

    def data_to_obj(self, data):
        # Converts the passed in data into an instance of the tuple, or returns None if this fails
        if len(data) != self.get_size():
            # Size mismatch!
            return None

        return self.named_tuple._make(struct.unpack(self.fmt, data))


# Define the data packets currently implemented
accel_data_packet = DataPacket(0, 'fff', 'x y z', name_override='AccelData')
gyro_data_packet = DataPacket(1, 'fff', 'gx gy gz', name_override='GyroData')
mag_data_packet = DataPacket(2, 'fff', 'x y z', name_override='MagData')
gps_data_packet = DataPacket(3, '?fffffffff', 'valid_fix latitude longitude height vN vE vD hAcc vAcc sAcc', name_override='MagData')


class DataLink:
    """
    New and improved class-based system for receiving telemetry.

    Implements "V2"/first generation binary protocol
    """

    def __init__(self, packet_filter=None, raw_mode=False):
        """
        Creates (but does not connect) a new data link.

        NOTE: Packet filtering is applied on the client - server will still send out everything

        :param packet_filter: Specifies a list of packet definitions to listen for / parse. Only enabled when raw_mode is False.
        :param raw_mode: Whether or not to return the "raw" data.
        """
        # Keep track of the inputted params; convert the packet filter data into a dict to make it easier + more
        # efficient to match to incoming packets
        self.raw_mode = raw_mode
        self.packet_filter = {}
        if packet_filter is not None:
            for packet in packet_filter:
                self.packet_filter[packet.get_id()] = packet

        # Sync pattern used between packets
        self.syncpattern = b'\xFF\x8C'

        # Header / first few bytes after the sync pattern but before the actual payload
        self.HeaderType = namedtuple('Header', 'pkt_id payload_size')
        self.header_fmt = 'BH'

        # Endianness/parsing and padding config
        # See: https://docs.python.org/3/library/struct.html#byte-order-size-and-alignment. Currently just storing
        # data as the STM32 stores it internally // using tight packing for custom values/outside of the payload -> uses
        # little endian by default
        self.system_format = '<'

        # CRC functionality to use (currently using CRC-CCITT)
        # NOTE: Probably the -FALSE version? At least, comparing a few values to https://crccalc.com/ would suggest that
        self.crc_fn = binascii.crc_hqx
        self.crc_size = 2

        # The (asyncio) connection
        self.reader = None
        self.writer = None

    async def connect(self):
        # Wireless connection
        # TODO: Make configurable eventually / potentially add back serial support?
        self.reader, self.writer = await asyncio.open_connection('192.168.1.246', 23)

        return self.reader is not None and self.writer is not None

    async def get_measurement(self):
        """
        Attempts to read a packet from the stream, and will return None if it fails.

        :return: Two cases:
            [raw_mode=False]: The parsed packet struct; stream is re-tried until we find a matching entry
            [raw_mode=True]: A tuple containing (pkt_id, payload_data) if successful, else None
        """
        # Read until the (next) sync pattern / discard any (irrelevant) data
        sync_data = await self.reader.readuntil(self.syncpattern)

        # Read the header
        header_data = await self.reader.readexactly(struct.calcsize(self.system_format + self.header_fmt))
        header = self.HeaderType._make(struct.unpack(self.system_format + self.header_fmt, header_data))

        # Read the payload
        payload_data = await self.reader.readexactly(header.payload_size)

        # Read the checksum
        pkt_checksum_data = await self.reader.readexactly(self.crc_size)
        pkt_checksum = int.from_bytes(pkt_checksum_data, byteorder='little')

        # Ensure the packet's checksum matches the one we're expecting
        # NOTE: 0xFFFF comes from INITIAL_REMAINDER
        data_to_checksum = header_data + payload_data
        computed_checksum = self.crc_fn(data_to_checksum, 0xFFFF)

        if computed_checksum != pkt_checksum:
            print(f"Checksum mismatch! (expected={pkt_checksum}; computed={computed_checksum}")
            print(f"Packet (' | ' as separator): {sync_data} | {header_data} | {payload_data} | {pkt_checksum_data}")
            print(f"Checksum over: {data_to_checksum.hex().upper()}")
            print(f"Header: {header}")
            print(f"~computed_checksum: {~computed_checksum}")
            print(f"Read Payload Size: {len(payload_data)}")
            return None

        # Return the data and packet ID if we're currently in RAW mode, else try to parse it / retry the measurement
        # / go to the next packet if it doesn't match the filter or is otherwise invalid
        if self.raw_mode:
            return header.pkt_id, payload_data

        packet_definition = self.packet_filter.get(header.pkt_id)
        if packet_definition:
            parsed_packet = packet_definition.data_to_obj(payload_data)
            if parsed_packet is not None:
                return parsed_packet

        # Unable to parse packet (some form of parsing error or it doesn't match the filter) - re-try the measurement
        # [using recursion because goto doesn't exist/this is easier than a loop]
        # TODO: Implement some form of maximum layers / exhaustion case? We don't want to have this go on "forever"
        #  (or until the max. recursion depth is reached)
        return await self.get_measurement()


# DataLink usage example / test
async def run_example():
    test_link = DataLink(packet_filter=[gps_data_packet], raw_mode=False)
    await test_link.connect()

    while True:
        measurement = await test_link.get_measurement()
        print(measurement)


if __name__ == '__main__':
    asyncio.get_event_loop().create_task(run_example())
    asyncio.get_event_loop().run_forever()
