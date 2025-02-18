# External module imports
import RPi.GPIO as GPIO
#from gpiozero import Robot, LineSensor
from time import sleep

# Pin Setup:
#---------------------------------------------
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme !!!!!!!!!!
IR_L_PIN = 16
IR_R_PIN = 26
IR_C_PIN = 4

GPIO.setup(IR_L_PIN, GPIO.IN)
GPIO.setup(IR_R_PIN, GPIO.IN)
GPIO.setup(IR_C_PIN, GPIO.IN)

#robot = Robot(left=(7, 8), right=(9, 10))
# left_sensor = LineSensor(17)
# right_sensor= LineSensor(27)

speed = 1

R_dir = 1
L_dir = 1

def motor_speed(L_dir,R_dir):
# while True:
    # left_detect  = int(left_sensor.value)
    # right_detect = int(right_sensor.value)
    # if left_detect == 0 and right_detect == 0:
    if (GPIO.input(IR_L_PIN) == 0) and (GPIO.input(IR_R_PIN) == 0) and (GPIO.input(IR_C_PIN) == 0):
        L_dir = 0
        R_dir = 0
        #print('forward')
    ## Stage 1
    # if left_detect == 0 and right_detect == 0:
    if (GPIO.input(IR_L_PIN) == 1) and (GPIO.input(IR_R_PIN) == 1) and (GPIO.input(IR_C_PIN) == 1):
        L_dir = 0
        R_dir = 0
        #print('forward')
    if (GPIO.input(IR_L_PIN) == 0) and (GPIO.input(IR_R_PIN) == 0) and (GPIO.input(IR_C_PIN) == 1):
        L_dir = 1
        R_dir = 1
        #print('forward')
    ## Stage 2
    #if left_detect == 0 and right_detect == 1:
    if (GPIO.input(IR_L_PIN) == 0) and (GPIO.input(IR_R_PIN) == 1) and (GPIO.input(IR_C_PIN) == 1):
        L_dir = 0#-1
        R_dir = 1
        #print('left')
    #if left_detect == 1 and right_detect == 0:
    if (GPIO.input(IR_L_PIN) == 1) and (GPIO.input(IR_R_PIN) == 0) and (GPIO.input(IR_C_PIN) == 1):
        L_dir = 1
        R_dir = 0#-1
        #print('right')
        ## Stage 3
    #if left_detect == 0 and right_detect == 1:
    if (GPIO.input(IR_L_PIN) == 0) and (GPIO.input(IR_R_PIN) == 1) and (GPIO.input(IR_C_PIN) == 0):
        L_dir = -1
        R_dir = 1
        #print('left')
    #if left_detect == 1 and right_detect == 0:
    if (GPIO.input(IR_L_PIN) == 1) and (GPIO.input(IR_R_PIN) == 0) and (GPIO.input(IR_C_PIN) == 0):
        L_dir = 1
        R_dir = -1
        #print('right')
    #print(R_mot, L_mot)
    return R_dir * speed, L_dir * speed

#robot.source = motor_speed()



# def main():
    # try:
        # while True:
            # R, L = motor_speed(L_dir,R_dir)
            # print(R, L)
    # except KeyboardInterrupt:
        # GPIO.cleanup()
        
# #### Main ####
# if __name__ == "__main__":
	# main()

# sleep(60)
# robot.stop()
# robot.source = None
# robot.close()
# left_sensor.close()
# right_sensor.close()
