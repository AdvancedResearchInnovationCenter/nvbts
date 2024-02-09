#to run without sudo, add your user to the dialout group
#sudo usermod -a -G tty your-username

import serial
import time
import glob

class Gate:
    def __init__(self):
        self.ser = serial.Serial(list(glob.glob('/dev/ttyUSB*'))[0]) #try ttyACM if this doesn't work
    
    def close(self):
        self.ser.write(b'c')

    def open(self):
        self.ser.write(b'o')