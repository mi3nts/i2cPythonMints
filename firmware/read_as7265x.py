from i2c_as7265x import AS7265X
import smbus2
import numpy as np
import matplotlib.pyplot as plt

bus = smbus2.SMBus(1)

debug  = False 

as7265x = AS7265X(bus,debug)

as7265x.begin()
as7265x.enableBulb(as7265x.LED_WHITE)
as7265x.enableBulb(as7265x.LED_IR)
as7265x.enableBulb(as7265x.LED_UV)
as7265x.setIntegrationCycles(1)

x = ['410', '435', '460', '485', '510', '535', '560', '585',
     '610', '645', '680', '705', '730', '760', '810', '860',
     '900', '940']

'''Alphabetical order is not spectral order. ie
A,B,C,D,E,F,G,H,I,J,K,L,R,S,T,U,V,W .
According to the data sheets, the spectral order is
A,B,C,D,E,F,G,H,R,I,S,J,T,U,V,W,K,L.

The order in the example reflects the UV to NIR spectral order.
'''

while (1):
    try:
        as7265x.takeMeasurements()

        data = []
        data.append(as7265x.getCalibratedA())
        data.append(as7265x.getCalibratedB())
        data.append(as7265x.getCalibratedC())
        data.append(as7265x.getCalibratedD())
        data.append(as7265x.getCalibratedE())
        data.append(as7265x.getCalibratedF())
        data.append(as7265x.getCalibratedG())
        data.append(as7265x.getCalibratedH())
        data.append(as7265x.getCalibratedR())        
        data.append(as7265x.getCalibratedI())
        data.append(as7265x.getCalibratedS())
        data.append(as7265x.getCalibratedJ())
        data.append(as7265x.getCalibratedT())
        data.append(as7265x.getCalibratedU())
        data.append(as7265x.getCalibratedV())
        data.append(as7265x.getCalibratedW())
        data.append(as7265x.getCalibratedK())
        data.append(as7265x.getCalibratedL())
        print(data)

    except:
        break

as7265x.disableBulb(as7265x.LED_WHITE)
as7265x.disableBulb(as7265x.LED_IR)
as7265x.disableBulb(as7265x.LED_UV)
