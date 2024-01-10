import serial
import time
ser = serial.Serial('/dev/ttyACM0')

# ser.write(b'o')
ser.write(b'c')

time.sleep(1) 