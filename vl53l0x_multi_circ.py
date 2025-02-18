# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Example of how to use the adafruit_vl53l0x library to change the assigned address of
multiple VL53L0X sensors on the same I2C bus. This example only focuses on 2 VL53L0X
sensors, but can be modified for more. BE AWARE: a multitude of sensors may require
more current than the on-board 3V regulator can output (typical current consumption during
active range readings is about 19 mA per sensor).
"""
import RPi.GPIO as GPIO
import time
import board
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
import busio

ENA_1 = 17
ENA_2 = 27
ENA_3 = 8
ENA_4 = 7

WAF_WEIGHT = 0.1	#0.2

GPIO.setmode(GPIO.BCM)

def ToF_init():
    # declare the singleton variable for the default I2C bus
    i2c = board.I2C()  # uses board.SCL and board.SDA
    # i2c = busio.I2C(board.SCL, board.SDA)
    # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
    
    # # declare the digital output pins connected to the "SHDN" pin on each VL53L0X sensor
    # xshut = [
        # DigitalInOut(board.D7),
        # DigitalInOut(board.D9),
        # # add more VL53L0X sensors by defining their SHDN pins here
    # ]
    
    # initialize a list to be used for the array of VL53L0X sensors
    vl53 = []
    #---------------------------------------------


    GPIO.setup(ENA_1, GPIO.OUT)#IN, pull_up_down=GPIO.PUD_OFF) # DIR pin set as output
    GPIO.setup(ENA_2, GPIO.OUT)#IN, pull_up_down=GPIO.PUD_OFF) # DIR pin set as output
    GPIO.setup(ENA_3, GPIO.OUT)#IN, pull_up_down=GPIO.PUD_OFF) # DIR pin set as output
    GPIO.setup(ENA_4, GPIO.OUT)#IN, pull_up_down=GPIO.PUD_OFF) # DIR pin set as output
    time.sleep(0.50)
    GPIO.output(ENA_1, GPIO.LOW) # LOW to enable
    GPIO.output(ENA_2, GPIO.LOW) # LOW to enable
    GPIO.output(ENA_3, GPIO.LOW) # LOW to enable
    GPIO.output(ENA_4, GPIO.LOW) # LOW to enable

    
    # for power_pin in xshut:
        # # make sure these pins are a digital output, not a digital input
        # power_pin.switch_to_output(value=False)
        # # These pins are active when Low, meaning:
        # #   if the output signal is LOW, then the VL53L0X sensor is off.
        # #   if the output signal is HIGH, then the VL53L0X sensor is on.
    # # all VL53L0X sensors are now off
    
    GPIO.output(ENA_1, GPIO.HIGH) # LOW to enable
    time.sleep(0.50)
    vl53.insert(0, VL53L0X(i2c))  # also performs VL53L0X hardware check
    # start continous mode
    vl53[0].start_continuous()
    # default address is 0x29. Change that to something else
    vl53[0].set_address(0+0x30)  # address assigned should NOT be already in use
    #GPIO.output(ENA_1, GPIO.LOW)
    vl53[0].measurement_timing_budget = 20000
    
    GPIO.output(ENA_2, GPIO.HIGH) # LOW to enable
    time.sleep(0.50)
    vl53.insert(1, VL53L0X(i2c))  # also performs VL53L0X hardware check
    # start continous mode
    vl53[1].start_continuous()
    # # default address is 0x29. Change that to something else
    vl53[1].set_address(1+0x30)  # address assigned should NOT be already in use
    #GPIO.output(ENA_2, GPIO.LOW)
    vl53[1].measurement_timing_budget = 20000
    
    GPIO.output(ENA_3, GPIO.HIGH) # LOW to enable
    time.sleep(0.50)
    vl53.insert(2, VL53L0X(i2c))  # also performs VL53L0X hardware check
    # start continous mode
    vl53[2].start_continuous()
    # default address is 0x29. Change that to something else
    vl53[2].set_address(2+0x30)  # address assigned should NOT be already in use
    #GPIO.output(ENA_1, GPIO.LOW)
    vl53[2].measurement_timing_budget = 20000
    
    GPIO.output(ENA_4, GPIO.HIGH) # LOW to enable
    time.sleep(0.50)
    vl53.insert(3, VL53L0X(i2c))  # also performs VL53L0X hardware check
    # start continous mode
    vl53[3].start_continuous()
    # # default address is 0x29. Change that to something else
    vl53[3].set_address(3+0x30)  # address assigned should NOT be already in use
    #GPIO.output(ENA_2, GPIO.LOW)
    vl53[3].measurement_timing_budget = 20000
    
    # # now change the addresses of the VL53L0X sensors
    # for i, power_pin in enumerate(xshut):
        # # turn on the VL53L0X to allow hardware check
        # power_pin.value = True
        # # instantiate the VL53L0X sensor on the I2C bus & insert it into the "vl53" list
        # vl53.insert(i, VL53L0X(i2c))  # also performs VL53L0X hardware check
        # # no need to change the address of the last VL53L0X sensor
        # if i < len(xshut) - 1:
            # # default address is 0x29. Change that to something else
            # vl53[i].set_address(i + 0x30)  # address assigned should NOT be already in use
    # # there is a helpful list of pre-designated I2C addresses for various I2C devices at
    # # https://learn.adafruit.com/i2c-addresses/the-list
    # # According to this list 0x30-0x34 are available, although the list may be incomplete.
    # # In the python REPR, you can scan for all I2C devices that are attached and detirmine
    # # their addresses using:
    # #   >>> import board
    # #   >>> i2c = board.I2C()
    # #   >>> if i2c.try_lock():
    # #   >>>     [hex(x) for x in i2c.scan()]
    # #   >>>     i2c.unlock()

    return vl53
    
def ToF_stop(vl53):
    """this is not required, if you use XSHUT to reset the sensor.
    unless if you want to save some energy
    """
    for sensor in vl53:
        vl53.stop_continuous()
    
def ToF_read(vl53,lastDistance):
    data = []
    currentDistance = []
    for index, sensor in enumerate(vl53):
        #print("LastDistnce: {}mm".format(lastDistance[index]))
        currentDistance.insert(index,float(WeightedAverageFilter(sensor.range, lastDistance[index])))
        #lastDistance[index] = currentDistance[index]
        data.insert(index, int(currentDistance[index]))
    return data
    
def WeightedAverageFilter(incomingValue,lastValue):
	result = float(WAF_WEIGHT * incomingValue + (1.0 - WAF_WEIGHT) * lastValue)
	return result  
    
def detect_range(vl53,count=5):
    """take count=5 samples"""
    while count:
        for index, sensor in enumerate(vl53):
            print("Sensor {} Range: {}mm".format(index + 1, sensor.range))
        time.sleep(0.1)
        count -= 1

print(
    "Multiple VL53L0X sensors' addresses are assigned properly\n"
    "execute detect_range() to read each sensors range readings"
)

# detect_range(count=50)
# # GPIO.output(ENA_1, GPIO.LOW)
# # GPIO.output(ENA_2, GPIO.LOW)
# GPIO.cleanup()
