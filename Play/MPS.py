#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

# Encoder and functions

import time
import os
import math
import spidev
import wiringpi
from Settings.MPS_CONTROL_TABLE import *

# Enable SPI
spi = spidev.SpiDev()


class MPS_Encoder(object):  # Handles a single encoder

    def __init__(self, name, chip_bus, cs, max_speed, mode, gpio=False):
        self.name = name
        if self.name == "MA310" or "MA710":
            self.precision = 12  # These are 12bit encoders
        else:
            self.precision = 12  # Default encoder precision
        self.chip_bus = chip_bus
        self.cs = cs  # Chip Select, set to 0 if connected to CE0 on Pi, or 1 if connected to CE1
        self.max_speed = max_speed
        self.mode = mode  # SPI mode, this should be just 0 for most MPS encoders.
        self.gpio = gpio  # Set to True if want to use GPIO as CS.
        if self.gpio:
            wiringpi.wiringPiSetupGpio()
            wiringpi.pinMode(self.cs, 1)
            wiringpi.digitalWrite(self.cs, 1)  # Set ChipSelect GPIO as HIGH

    def connect(self):
        # Open a connection to a specific bus and device (chip select pin)
        if self.gpio:
            cs = 2  # Using GPIO pins, thus set cs as something not existing
            spi.open(self.chip_bus, cs)
            wiringpi.digitalWrite(self.cs, 0)  # Set ChipSelect GPIO as LOW
        else:
            spi.open(self.chip_bus, self.cs)
        # Set SPI speed and mode
        spi.max_speed_hz = self.max_speed
        spi.mode = self.mode
        print("SPI device connected.")

    def read_angle(self):
        # Read angle from device
        data = spi.readbytes(2)
        high_byte = data[0] << 8
        low_byte = data[1]
        angle = (high_byte + low_byte) >> 4  # Get rid of last 4 bit whatever
        return angle

    def read_BCT(self):
        # Read the BCT register value
        send = 0b01000010
        spi.writebytes([send, 0])
        data = spi.readbytes(2)
        BTC = data[0]
        return BTC

    def write_BTC(self, BTC):
        # Write the BCT register value
        # BTC value
        send = 0b10000010
        spi.writebytes([send, BTC])
        time.sleep(0.02)
        data = spi.readbytes(2)
        high_byte = data[0]
        if high_byte == BTC:
            return True
        else:
            return False

    def release(self):
        # Disconnect the device
        if self.gpio:
            wiringpi.digitalWrite(self.cs, 1)  # Set ChipSelect GPIO as HIGH
        spi.close()
        print("SPI device released.")

    def read_reg(self, reg_name):
        # Read from a register
        packet = INSTRUCTION.read + REG_DIC[reg_name]
        spi.writebytes([packet, 0])
        data = spi.readbytes(2)
        reg_val = data[0]
        return reg_val

    def write_reg(self, reg_name, reg_val):
        # Write to a register
        packet = INSTRUCTION.write + REG_DIC[reg_name]
        spi.writebytes([packet, reg_val])
        time.sleep(0.02)
        data = spi.readbytes(2)
        return_val = data[0]
        if return_val == reg_val:
            return True
        else:
            return False
        
    def home(self):
        # Zero the sensor with current position
        # Read current position:
        data = spi.readbytes(2)
        high_byte = data[0] << 8
        low_byte = data[1]
        angle = high_byte + low_byte  
        a0 = 65536 - angle
        a0_h = a0 >> 8
        a0_l = a0 & 0b0000000011111111
        
        check = [0,0]
        packet = INSTRUCTION.write + REG_DIC['zero_high']
        spi.writebytes([packet, a0_h])
        time.sleep(0.02)
        data = spi.readbytes(2)
        return_val = data[0]
        if return_val == a0_h:
            check[0] = 1
        else:
            print("High byte update failed.")
        
        packet = INSTRUCTION.write + REG_DIC['zero_low']
        spi.writebytes([packet, a0_l])
        time.sleep(0.02)
        data = spi.readbytes(2)
        return_val = data[0]
        if return_val == a0_l:
            check[1] = 1
        else:
            print("Low byte update failed.")
        
        if check[1] and check[0]:
            print("Homed.")
            return True
        else:
            return False


class MPS_Encoder_Cluster(object):  # Handles a cluster of encoders via utilizing GPIO pins as CS

    def __init__(self, name, chip_bus, cs, max_speed, mode):
        self.name = name
        if self.name == "MA310" or "MA710":
            self.precision = 4096  # These are 12bit encoders
        else:
            self.precision = 4096  # Default encoder precision
        self.conversion = 2*math.pi/self.precision  # Conversion rate from encoder reading to [0, 2*pi)
        self.chip_bus = chip_bus
        self.cs = cs  # Chip Select, a list of CS pins on GPIO
        self.max_speed = max_speed
        self.mode = mode  # SPI mode, this should be just 0 for most MPS encoders.
        # Initiate GPIO pins
        wiringpi.wiringPiSetupGpio()
        for i in self.cs:
            wiringpi.pinMode(i, 1)  # Set all ChipSelect GPIO as output
            wiringpi.digitalWrite(i, 1)  # Set ChipSelect GPIO as HIGH
        self.angles = [None]*len(self.cs)  # Initiate angles

    def connect(self):
        # Open a connection to a specific bus
        spi.open(self.chip_bus, 0)  # Using GPIO pins tho
        spi.max_speed_hz = self.max_speed
        spi.mode = self.mode
        print("SPI devices connected.")

    def read_angle(self):
        # Read angle from all devices
        for idx, val in enumerate(self.cs):
            wiringpi.digitalWrite(val, 0)  # Set target ChipSelect GPIO as LOW
            data = spi.readbytes(2)
            wiringpi.digitalWrite(val, 1)  # Set target ChipSelect GPIO as HIGH
            high_byte = data[0] << 8
            low_byte = data[1]
            self.angles[idx] = ((high_byte + low_byte) >> 4) * self.conversion
            # Get rid of last 4 bit whatever and convert to rad
        return self.angles

    def release(self):
        # Disconnect the device
        spi.close()
        for i in self.cs:
            wiringpi.digitalWrite(i, 1)  # Set all ChipSelect GPIO as HIGH
        print("SPI devices released.")


