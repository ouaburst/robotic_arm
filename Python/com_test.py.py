# Import packages
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import argparse
import sys

# I2C
import smbus
import time
import sys
bus = smbus.SMBus(1)
address = 0x04              # Arduino I2C Address


while(True):

    bus.write_byte(address,2)

    if bus.read_byte(address) == 3:
		break


	time.sleep(1) 