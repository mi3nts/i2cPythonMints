#!/usr/bin/python
# ***************************************************************************
#   I2CPythonMints
#   ---------------------------------
#   Written by: Lakitha Omal Harindha Wijeratne
#   - for -
#   MINTS :  Multi-scale Integrated Sensing and Simulation
#     & 
#   TRECIS: Texas Research and Education Cyberinfrastructure Services
#
#   ---------------------------------
#   Date: July 7th, 2022
#   ---------------------------------
#   This module is written for generic implimentation of MINTS projects
#   --------------------------------------------------------------------------
#   https://github.com/mi3nts
#   https://trecis.cyberinfrastructure.org/
#   http://utdmints.info/
#  ***************************************************************************


#import SI1132
from i2c_scd30 import SCD30

import sys
import time
import os
import smbus2



bus = smbus2.SMBus(1)
scd30 = SCD30(bus)


def main():
    scd30_valid  = scd30.initiate_scd30(30)

    while True:
        os.system('clear')
        if scd30_valid:
            if scd30.get_data_ready():
                measurement = scd30.read_measurement()
                if measurement is not None:
                    co2, temp, rh = measurement
                    print(f"CO2: {co2:.2f}ppm, temp: {temp:.2f}'C, rh: {rh:.2f}%")
        
        
        
        time.sleep(5)
        
        # print("======== bme280 ========")
        # print("temperature : %.2f 'C" % bme280.read_temperature())
        # print("humidity : %.2f %%" % bme280.read_humidity())
        # p = bme280.read_pressure()
        # print("pressure : %.2f hPa" % (p / 100.0))
        # print("altitude : %.2f m" % get_altitude(p, 1024.25))
        # print("======== ====== ========")
        # time.sleep(1)


    # if len(sys.argv) != 2:
    #     print("ADD I2C DEV ADDRESS ARG")
    #     sys.exit()

    # #si1132 = SI1132.SI1132(sys.argv[1])
    # bme280 = i2c_bme280.BME280(sys.argv[1], 0x03, 0x02, 0x02, 0x02)

    # def get_altitude(pressure, seaLevel):
    #     atmospheric = pressure / 100.0
    #     return 44330.0 * (1.0 - pow(atmospheric/seaLevel, 0.1903))


if __name__ == "__main__":
   main()