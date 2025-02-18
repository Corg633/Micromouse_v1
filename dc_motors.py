# External module imports
import RPi.GPIO as GPIO
import time

# Pin Definitons:
# LEFT_MOTOR
#---------------------------------------------
#global pwm1
# Configuration
FREQ_L = 100
#pwmHoriz = 18#13 # Broadcom pin 18 (P1 pin 13)
pwmL0 = 24
pwmL1 = 25

# ENA_L = 27 # HIGH to EN
# #DIR_H = 25 # Broadcom pin 23 (P1 pin 16)

#SW_RIGHT = 22 # Broadcom pin 17 (P1 pin 11)
#SW_LEFT = 23 # Broadcom pin 17 (P1 pin 11)

DC_L = 50 # duty cycle (0-100) for PWM pin

#---------------------------------------------

# RIGHT_MOTOR
#---------------------------------------------
#global pwm1
# Configuration
FREQ_R = 100
#pwmHoriz = 18#13 # Broadcom pin 18 (P1 pin 13)
pwmR0 = 22
pwmR1 = 23

# ENA_R = 17 # HIGH to EN
# #DIR_H = 25 # Broadcom pin 23 (P1 pin 16)

#SW_RIGHT = 22 # Broadcom pin 17 (P1 pin 11)
#SW_LEFT = 23 # Broadcom pin 17 (P1 pin 11)

DC_R = 50 # duty cycle (0-100) for PWM pin

#---------------------------------------------

# Pin Setup:
#---------------------------------------------
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme !!!!!!!!!!
#time.sleep(1)
#---------------------------------------------

# MOTOR_L Init
#---------------------------------------------
def setup():

	# #GPIO.setup(DIR_H, GPIO.OUT) #IN, pull_up_down=GPIO.PUD_UP) # DIR pin set as output
	# GPIO.setup(ENA_L, GPIO.OUT)#IN, pull_up_down=GPIO.PUD_OFF) # DIR pin set as output
	
	#PWM0 Initialize (1kHz)
	GPIO.setup(pwmL0, GPIO.OUT) # PWM pin set as output
	pwm0 = GPIO.PWM(pwmL0, FREQ_L)  # Initialize PWM on pwmPin 1kHz frequency
	
	#PWM1 Initialize (1kHz)
	GPIO.setup(pwmL1, GPIO.OUT) # PWM pin set as output
	pwm1 = GPIO.PWM(pwmL1, FREQ_L)  # Initialize PWM on pwmPin 1kHz frequency
	
	#GPIO.setup(SW_LEFT, GPIO.IN)#,pull_up_down=GPIO.PUD_OFF)
	#GPIO.setup(SW_RIGHT, GPIO.IN)#,pull_up_down=GPIO.PUD_OFF)
	
	#---------------------------------------------
	
	# MOTOR_R Init
	#---------------------------------------------
	
	# #GPIO.setup(DIR_H, GPIO.OUT) #IN, pull_up_down=GPIO.PUD_UP) # DIR pin set as output
	# GPIO.setup(ENA_R, GPIO.OUT)#IN, pull_up_down=GPIO.PUD_OFF) # DIR pin set as output
	
	#PWM0 Initialize (1kHz)
	GPIO.setup(pwmR0, GPIO.OUT) # PWM pin set as output
	pwm2 = GPIO.PWM(pwmR0, FREQ_R)  # Initialize PWM on pwmPin 1kHz frequency
	
	#PWM1 Initialize (1kHz)
	GPIO.setup(pwmR1, GPIO.OUT) # PWM pin set as output
	pwm3 = GPIO.PWM(pwmR1, FREQ_R)  # Initialize PWM on pwmPin 1kHz frequency
	
	#GPIO.setup(SW_LEFT, GPIO.IN)#,pull_up_down=GPIO.PUD_OFF)
	#GPIO.setup(SW_RIGHT, GPIO.IN)#,pull_up_down=GPIO.PUD_OFF)
	
	#---------------------------------------------
	
	# Initial state for Motors:
	#---------------------------------------------
	
	# #GPIO.output(DIR_H, GPIO.LOW) # HIGH - left / LOW - right
	# GPIO.output(ENA_L, GPIO.LOW) # LOW to enable
	
	# #GPIO.output(DIR_H, GPIO.LOW) # HIGH - left / LOW - right
	# GPIO.output(ENA_R, GPIO.LOW) # LOW to enable
	
	pwm0.start(0)	# L
	pwm1.start(0)	# L
	pwm2.start(0)	# R
	pwm3.start(0)	# R
	
	return pwm0,pwm1,pwm2,pwm3

#------------------------------------

def hor_stepper(dir,x):
	if (dir == 'left') :
		GPIO.output(ENA_H, GPIO.LOW)
		GPIO.output(DIR_H, GPIO.HIGH)	# HIGH - left / LOW - right
		t0 = time.time()
		diff = 0
		while (diff <= x):
			if (GPIO.input(22) == 1) : # right swith pressed
				GPIO.output(DIR_H, GPIO.HIGH) # turn left
			if (GPIO.input(23) == 1) : # left swith pressed
				GPIO.output(DIR_H, GPIO.LOW) # turn right
			else: # button not pressed:
				GPIO.output(ENA_H, GPIO.LOW)  #LOW TO enable
				t1 = time.time()
				diff = t1 - t0
				#time.sleep(x)
			
	if (dir == 'right') :
		GPIO.output(ENA_H, GPIO.LOW)
		GPIO.output(DIR_H, GPIO.LOW)	# HIGH - left / LOW - right
		t0 = time.time()
		diff = 0
		while (diff <= x):
			if (GPIO.input(22) == 1) : # right swith pressed
				GPIO.output(DIR_H, GPIO.HIGH) # turn left
			if (GPIO.input(23) == 1) : # left swith pressed
				GPIO.output(DIR_H, GPIO.LOW) # turn right
			else: # button not pressed:
				GPIO.output(ENA_H, GPIO.LOW)  #LOW TO enable
				t1 = time.time()
				diff = t1 - t0
				#time.sleep(x)
			
	if (dir == 'left_align') :
		GPIO.output(ENA_H, GPIO.LOW)
		GPIO.output(DIR_H, GPIO.HIGH)	# HIGH - left / LOW - right
		while (GPIO.input(23) == 0):
			if (GPIO.input(22) == 1) : # right swith pressed
				GPIO.output(DIR_H, GPIO.HIGH) # turn left
			if (GPIO.input(23) == 1) : # left swith pressed
				GPIO.output(DIR_H, GPIO.LOW) # turn right
			else: # button not pressed:
				GPIO.output(ENA_H, GPIO.LOW)  #LOW TO enable
				#time.sleep(x)
				
	if (dir == 'right_align') :
		GPIO.output(ENA_H, GPIO.LOW)
		GPIO.output(DIR_H, GPIO.LOW)	# HIGH - left / LOW - right
		while (GPIO.input(22) == 0):
			if (GPIO.input(22) == 1) : # right swith pressed
				GPIO.output(DIR_H, GPIO.HIGH) # turn left
			if (GPIO.input(23) == 1) : # left swith pressed
				GPIO.output(DIR_H, GPIO.LOW) # turn right
			else: # button not pressed:
				GPIO.output(ENA_H, GPIO.LOW)  #LOW TO enable
				#time.sleep(x)
	GPIO.output(ENA_H, GPIO.HIGH)

def hor_stepper_stop():
	GPIO.output(ENA_H, GPIO.HIGH) # LOW to enable
	pwm1.stop() # stop PWM
	#GPIO.cleanup() # cleanup all GPIO
#------------------------
	
def run_L(pwm0,pwm1):
	GPIO.output(ENA_L, GPIO.HIGH) # LOW to enable
	pwm0.ChangeDutyCycle(0)
	pwm1.ChangeDutyCycle(50)
	
def run_R(pwm2,pwm3):
	GPIO.output(ENA_R, GPIO.HIGH) # LOW to enable
	pwm2.ChangeDutyCycle(50)
	pwm3.ChangeDutyCycle(0)
	
def runM(pwm0,pwm1,pwm2,pwm3,DC0,DC1,DC2,DC3):
	# GPIO.output(ENA_L, GPIO.HIGH) # LOW to enable
	# GPIO.output(ENA_R, GPIO.HIGH) # LOW to enable
	pwm0.ChangeDutyCycle(DC0)
	pwm1.ChangeDutyCycle(DC1)
	pwm2.ChangeDutyCycle(DC2)
	pwm3.ChangeDutyCycle(DC3)
	
def stopM(pwm0,pwm1,pwm2,pwm3):
	# GPIO.output(ENA_L, GPIO.LOW) # LOW to disable
	# GPIO.output(ENA_R, GPIO.LOW) # LOW to disable
	pwm0.stop() # stop PWM
	pwm1.stop() # stop PWM
	pwm2.stop() # stop PWM
	pwm3.stop() # stop PWM
	
	#GPIO.cleanup()
	#time.sleep(1)
	#GPIO.setmode(GPIO.BCM)
	#GPIO.cleanup() # cleanup all GPIO

# ----main -----
# print("Here we go! Press CTRL+C to exit")
# try:
	# while True:
	# #hor_stepper('left',1)
	# #hor_stepper('right',1)
	# #hor_stepper('left_align',1)
	# #hor_stepper('right_align',1)
	
		# run_L()
		# run_R()
	
	# #time.sleep(5)
	# #hor_stepper_stop()
	# #stop()


# except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
	# #hor_stepper_stop()
	# stop()
# #finally:
	# #hor_stepper_stop()
	# #stop()
