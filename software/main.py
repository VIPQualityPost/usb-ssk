import serial
import struct

serialPort = "/dev/tty.usbmodem2081347756501"
serialBaud = 115200

ser = serial.Serial(serialPort, serialBaud)

while(1):
    accel_data = ser.read(4)
    float_data = struct.unpack('>f', accel_data)
    # accel_data = ser.read(12)

    # accel_x = accel_data & 0xFFFFFFFF0000000000000000
    # accel_y = accel_data & 0x00000000FFFFFFFF00000000
    # accel_z = accel_data & 0x0000000000000000FFFFFFFF
    print(float_data, accel_data)

