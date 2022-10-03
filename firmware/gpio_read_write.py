
import odroid_wiringpi as wpi
import time
import sys
import os
import smbus2
#import SI1132

from i2c_scd30 import SCD30
#from i2c_as7265x import AS7265X
from i2c_bme280 import BME280
wpi.wiringPiSetup()

debug  = False 

bus     = smbus2.SMBus(1)

scd30   = SCD30(bus,debug)
#as7265x = AS7265X(bus,debug)
bme280  = BME280(bus,debug)

scd30_valid    = scd30.initiate(30)
#    as7265x_valid  = as7265x.initiate()
bme280_valid   = bme280.initiate(30)

while True:
    try:
        time.sleep(2.5)
        print("=======================")
        print("======== SCD30 ========")
        if scd30_valid:
            scd30.read()
        print("=======================")
        time.sleep(2.5)
        
        print("======= BME280 ========")
        if bme280_valid:
            bme280.read()
        print("=======================")
        time.sleep(2.5)        
      
        batteryLevelRaw = wpi.analogRead(25)
        time.sleep(1.5)
    
        batteryLevel    = batteryLevelRaw*(2.1*2)/(4095)
        batteryLevelPer = batteryLevel*(100/4.2)
 
        referenceLev    = wpi.analogRead(29)
        time.sleep(1)
        print("======= Battery Readings =======")
        print("Reference Level:          " + str(referenceLev))
        print("---------------------------------")
        print("Battery Level Raw:        " + str(batteryLevelRaw))
        print("---------------------------------")
        print("Battery Level:            " + str(batteryLevel)+" V")
        print("---------------------------------")
        print("Battery Level Percentage: " + str(batteryLevelPer)+" %")    
    except Exception as e:
        print(e)
        break