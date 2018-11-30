import raspberry_bme280 as Bosch
import time

DEGREE_SIGN= u'\N{DEGREE SIGN}'

sensor=Bosch.BME280(1)
sym_id=0
SYMBOLS = ('|','/','|','\\')

try:
    while True:
        sensor.set_mode(sensor.MODE_FORCED)

        time.sleep(0.040)

        temp, press, humid=sensor.readSensor(sensor.READ_ALL)
        symbol = SYMBOLS[sym_id]
        
        print("  %2.2f%sC\t %d millibar\t %2.2f%% %s" % \
              (temp, DEGREE_SIGN, press, humid, symbol),end='     \r')
        time.sleep(2)

        sym_id = (sym_id + 1) % len(SYMBOLS)
        
except KeyboardInterrupt:
    pass

print("")
