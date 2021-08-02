import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
import serial
import struct
import time
from collections import namedtuple

# Some UBX protocol constants
UBX_SYNC_1 = 0xB5  # Sync pattern
UBX_SYNC_2 = 0x62

# Some UBX protocol helpers
def get_data(msg_class, msg_id, payload):
    data = []

    payload_data = bytes(payload)
    payload_length = len(payload_data) & 0xFFFF  # TODO: Eval bitmask - we want to avoid this being more than two bytes
    #data.append(UBX_SYNC)
    #data.append(msg_class)
    #data.append(msg_id)
    #data.append(payload_length)
    header_bytes = bytes(struct.pack("BBBBH", UBX_SYNC_1, UBX_SYNC_2, msg_class, msg_id, payload_length))
    #data.extend(header_bytes)
    #data.extend(payload_data)
    data = list(header_bytes + payload_data)
    #print(data)
    #print(list(data))
    #print('--')
    #print(header_bytes)
    #print(payload_data)
    #print(b''.join([header_bytes, payload_data]))

    # NOTE: This bytes call just makes things easier, but could possibly be optimized away in the future
    data_bytes = bytes(data)
    #print(data_bytes)

    # Compute the checksum mover the data - should include everything other than the sync pattern (hence starting at
    # pos 2) and the checksum bytes
    CK_A, CK_B = 0, 0
    for i in range(2, len(data_bytes)):
        CK_A += data_bytes[i]
        CK_B += CK_A

    # Add the checksum to the packet
    data.append(CK_A & 0xFF)
    data.append(CK_B & 0xFF)

    # Return the resulting bytes!
    return bytes(data)


def format_hex(bytes):
    """
    Helper function that formats the bytes into an easy-to-read string
    :param bytes:
    :return:
    """
    return bytes.hex(' ').upper()


# NOTE: Apparently device is LSB!
#print(structuin.pack('I', 0b00000000000000000000100011000000))
CFG_PRT_REQ = struct.pack('BBHIIHHHH', *[0x01, 0x00, 0b0000000000000000, 0b00000000000000000000100011010000, 115200, 0b0000000000000001, 0b0000000000000001, 0x00, 0x00])

# According to
# https://www.u-blox.com/sites/default/files/products/documents/UBX-G7020_ProductSummary_%28UBX-13003349%29.pdf,
# the max. nav rate is 10Hz....so use it
CFG_RATE_REQ = struct.pack('HHH', 100, 1, 1)
#print(CFG_PRT_REQ)
#CFG_PRT_REQ = [0x01, 0x00, 0b0000000000000000, 0b00000000000000000000100011000000, 0x0001B774, 0b0000000000000001, 0b0000000000000001, 0x00]
#print(get_data(0x06, 0x00, CFG_PRT_REQ))

print(format_hex(get_data(0x06, 0x00, CFG_PRT_REQ)))

# Initial serial connection - will perform configuration
with serial.Serial('/dev/tty.usbserial-A900HIOM', 9600) as ser:
    packet = get_data(0x06, 0x00, CFG_PRT_REQ)#CFG_PRT_REQ)
    print(len(packet))
    print(packet.hex())
    ser.write(packet)
    time.sleep(0.5)
    #ser.read_until(b'\xB5\x62')
    print(f"Response: {ser.read_all()}")
    ser.close()

time.sleep(1)

with serial.Serial('/dev/tty.usbserial-A900HIOM', 115200) as ser:
    # Apparently it nno longer auto-sends position fixes, so quick hacky way to at least read some data to see if everythinng worked the way it should
    time.sleep(0.5)
    print("Sending PRT config fetch packet...")
    ser.write(get_data(0x06, 0x00, []))
    time.sleep(0.1)
    print(f"Result: {format_hex(ser.read_all())}")

    print("Sending CFG-MSG poll packet...")
    ser.write(get_data(0x06, 0x01, [0x01, 0x07]))
    time.sleep(0.1)
    print(f"Result: {format_hex(ser.read_all())}")

    print("Sending CFG-MSG update packet...")
    # NOTE: Last # is every how many epochs/nav cycles it should be sent
    ser.write(get_data(0x06, 0x01, [0x01, 0x07, 0x05]))
    time.sleep(0.1)
    print(f"Result: {format_hex(ser.read_all())}")

    print("Sending CFG-RATE poll packet...")
    ser.write(get_data(0x06, 0x08, []))
    time.sleep(0.1)
    print(f"Result: {format_hex(ser.read_all())}")

    # See later comment - delayed triggering of tweaking the navupdate rate; this variable ensures we only send the
    # packet once
    updated_rate = False
    
    while True:
        data = ser.read_all()

        # Is this a NAV-PVT packet?
        if data[:4] == b'\xB5\x62\x01\x07':
            packettuple = namedtuple('NAVPVT', 'iTOW year month day hour min sec valid tAcc nano fixType flags reserved1 numSV lon lat height hMSL hAcc vAcc velN velE velD gSpeed heading sAcc headingAcc pDOP reserved2 reserved3')
            #print(len(data))
            try:
                res = packettuple._make(struct.unpack('IHBBBBBBIiBBBBiiiiIIiiiiiIIHHI', data[6:84+6]))
                print(res)

                # NOTE: Getting a fix at the higher update rate seems almost impossible (upwards of 20 mins without a
                # fix while at the "default"/1000ms update rate it's OK - likely due to overloading the CPU?
                # In any case, that's why we update the rate after the fix resolves to a non-zero (-> assumed to be
                # valid? though there are edge cases here) fix
                if res.valid and not updated_rate:
                    print("Sending CFG-RATE update packet...")
                    ser.write(get_data(0x06, 0x08, CFG_RATE_REQ))
                    time.sleep(0.1)
                    print(f"Result: {format_hex(ser.read_all())}")
                    updated_rate = True
            except struct.error:
                # Skip malformed packets
                continue

        time.sleep(0.1)
