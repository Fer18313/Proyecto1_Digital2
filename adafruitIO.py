# Código de ejemplo AdafruitIO
# Universidad del Valle de Guatemala
# IE3027 - Electrónica Digital 2
# Diego Morales

# Adafruit IO
# https://io.adafruit.com/

# if Module not Found. Open Terminal/CMD and execute:
# pip3 install Adafruit_IO

from Adafruit_IO import Client, RequestError, Feed
import serial
import time
ADAFRUIT_IO_USERNAME = "Fernando18313"
ADAFRUIT_IO_KEY = "aio_SDEv058Ml06emzp4bbZVKObVJgUw"
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

#Digital Feed

port = serial.Serial("COM4",9600)
port.timeout = 3
time.sleep(1)
print('PORT INITIALIZED CORRECTLY')

digitalfeed = aio.feeds("temperature")
digitalfeed2 = aio.feeds("humidity")
digitalfeed3 = aio.feeds("ldr")
        
def getvalue():
    start = b'\x00'
    port.write(start)
    unit0 = port.readline().decode('ascii')
    dec0 = port.readline().decode('ascii')
    unit1 = port.readline().decode('ascii')
    dec1 = port.readline().decode('ascii')
    ldr = port.readline().decode('ascii')
    time.sleep(0.01)
    join =unit0 + dec0
    return join

while(1):
    data = getvalue()
    data2 = getvalue()
    data3 = getvalue()
    aio.send_data(digitalfeed.key,data)
    aio.send_data(digitalfeed2.key,data2)
    aio.send_data(digitalfeed3.key,data3)
    print(data)
    time.sleep(0.01)
