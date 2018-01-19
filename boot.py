# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal
from machine import UART
import os
from network import WLAN

uart = UART(0, 115200)
os.dupterm(uart)
wlan = WLAN()
wlan.init(mode=WLAN.AP, ssid='WIPY-PC', auth=(WLAN.WPA2, 'password'),channel=6,antenna=WLAN.INT_ANT)
#execfile('solar_controller.py')
#execfile('EssaiNewOnewire.py')
