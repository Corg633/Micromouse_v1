# External module imports
import RPi.GPIO as GPIO
import pigpio
import time
#------------
from dc_motors import *
#from matplot import *
from ir_sense import *
from pid_c import PID
# from VL53L0X_multi import *
from vl53l0x_multi_circ import *
#from test import *
#from encoder import Encoder
from MinIMU_v5_pi import MinIMU_v5_pi
#------------
from multiprocessing import Process, Lock, Pool
import multiprocessing as mp
#------------
import pigpio
from rotary_encoder import decoder
import rotary_encoder
#------------
import numpy as np
#from simple_pid import PID
#------------
pos_L = 0
pos_R = 0
#------------
way_L = 0
way_R = 0

wcurr_L = 0
output_L = 0
wcurr_R = 0
output_R = 0

# Setting general parameters
tstop = 2  # Execution duration (s)
tsample = 0.01  # Sampling period (s)
#wsp = 20  # Motor speed set point (rad/s)
#tau = 0.1  # Speed low-pass filter response time (s)
taupid = 0.1

j = 0
datar = []
datar_old = []

BUTTON = 18



def callback_L(way_L):
	global pos_L
	pos_L += way_L
	#print("L={}".format(pos_L))

def callback_R(way_R):
	global pos_R
	pos_R += way_R
	#print("R={}".format(pos_R))
  
#-----------------------------------------------------------------  

def button_callback(button):
    if GPIO.input(button) == GPIO.LOW:
        print("pressed.")
        state = 1
    else:
        print("released.")
        state = 0
    return state
        
def button_init(button):
	GPIO.setwarnings(True)        
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	#GPIO.add_event_detect(button, GPIO.BOTH, callback=button_callback, bouncetime=10)
	#time.sleep(1)
	
	
# def button_callback_pigpio(gpio): 
	# if gpio == self.gpioA:
		# print("pressed.")
	# else:
		# print("released.")	
	
# def button_init_pigpio(pi): 	
	# gpioA = 18
	# pi.set_mode(gpioA, pigpio.INPUT)

	# pi.set_pull_up_down(gpioA, pigpio.PUD_UP)

	# cbA = pi.callback(gpioA, pigpio.EITHER_EDGE, button_callback_pigpio)

def runR(j,sense,IMU,pid_L,pid_R,pwm0,pwm1,pwm2,pwm3):

	
	# Initializing previous and current values
	#ucurr = 0  # x[n] (step input)
	wfprev_L = 0  # y[n-1]
	wfcurr_L = 0  # y[n]
	wfprev_R = 0  # y[n-1]
	wfcurr_R = 0  # y[n]
	
	# Initializing variables and starting clock
	thetaprev_L = 0
	thetaprev_R = 0
	tprev = 0
	tcurr = 0
	tprev1 = 0
	tcurr1 = 0
	
	tstart = time.perf_counter()

	DC0 = 0
	DC1 = 0
	DC2 = 0
	DC3 = 0
	
	DEF1=-20
	DEF2=20
	
	#Factor = 4
	i = 0
	
	L = 1
	R = 1
	
	Gx = 0
	Gy = 0
	Gz = 0
	
	datar_old.insert(0,0)
	datar_old.insert(1,0)
	datar_old.insert(2,0)
	datar_old.insert(3,0)
	#num = mp.cpu_count()
	#pool = mp.Pool(num)
	
	f1 = 0
	o = 0
	o1 = 0
	prev_state = 0
	countCh = 0
	stop = 0
	while True:
		i = i+1
		j = j+1
		
		#-------------------------
		
		# Getting current time (s)
		tcurr = time.perf_counter() - tstart
		dt = (tcurr-tprev)
		tprev = tcurr
		#print (dt)
		

		# Getting motor shaft angular position: I/O (data in)
		thetacurr_L = pos_L
		thetacurr_R = pos_R
		
		# Calculating motor speed (rad/s)
		wcurr_L = np.pi/180 * (thetacurr_L-thetaprev_L)/dt
		wcurr_R = np.pi/180 * (thetacurr_R-thetaprev_R)/dt
		
		# Filtering motor speed signal
		#wfcurr_L = tau/(tau+tsample)*wfprev_L + tsample/(tau+tsample)*wcurr_L
		#wfcurr_R = tau/(tau+tsample)*wfprev_R + tsample/(tau+tsample)*wcurr_R
		
		# wfprev_L = wfcurr_L
		# wfprev_R = wfcurr_R
		
		# Updating previous values
		thetaprev_L = thetacurr_L
		thetaprev_R = thetacurr_R
		# -------------
		
		# print("L= {}".format(int(wcurr_L)))
		# print("R= {}".format(int(wcurr_R)))
		
		print("L= {}    R= {}".format(int(wcurr_L),int(wcurr_R)))
		
		#------------
		# # Calculating closed-loop output
		# output_L = pid_L(wcurr_L-L*DEF1)		
		# output_R = pid_R(wcurr_R-R*DEF2)
		#------------
		output_L = pid_L.control(0,wcurr_L-L*DEF1)		
		output_R = pid_R.control(0,wcurr_R-R*DEF2)
		#------------
		
		# #m1 = mp.Process(target=runM, args = (pwm0,pwm1,pwm2,pwm3,DC0,DC1,DC2,DC3))
		# # m2 = mp.Process(target=anim_plot, args = (j, wcurr_L, output_L, wcurr_R, output_R))
		# m1 = mp.Process(target=pid_L.control,args =(0,wcurr_L-L*DEF1))
		# m2 = mp.Process(target=pid_R.control,args =(0,wcurr_R-R*DEF2))
		# #m3 = mp.Process(target=ToF_read, args = (sense1,sense2))
		
		# m1.start()
		# m2.start()
		# #m3.start()
		
		# m1.join()
		# m2.join()
		# #m3.join()
		#------------
		
		#output_L = pid_L.ucurr
		#output_R = pid_R.ucurr
		
		#print("output_L={}".format(output_L))
		#print("output_R={}".format(output_R))
		
		# flip it and -1
		if (output_L > 0):
			#output_L = pid_L.update(control_L)
			DC0 = int(output_L)
			DC1 = 0
		if (output_R < 0):
			#output_R = pid_L.update(control_R)
			DC2 = 0
			DC3 = int(-output_R)
		if (output_L < 0):
			#output_L = pid_L.update(control_L)
			DC0 = 0
			DC1 = int(-output_L	)		
		if (output_R > 0):
			#output_R = pid_L.update(control_R)
			DC2 = int(output_R)
			DC3 = 0
		if (output_L == 0):
			#output_L = pid_L.update(control_L)
			DC0 = 0
			DC1 = 0
		if (output_R == 0):
			#output_R = pid_L.update(control_R)
			DC2 = 0
			DC3 = 0
		
		print("DC{}: {} DC{}: {} DC{}: {} DC{}: {}".format(1,DC0,2,DC1,3,DC2,4,DC3))
		
		# -------------
		if (f1 == 5):
			runM(pwm0,pwm1,pwm2,pwm3,0,0,0,0)
		else:
			runM(pwm0,pwm1,pwm2,pwm3,DC0,DC1,DC2,DC3)
		#L, R = motor_speed(L_dir,R_dir)
		
		# pid_L = PID(P, I, D, setpoint=L*50*Factor)
		# pid_R = PID(P, I, D, setpoint=-R*50*Factor)
		
		# ------------------------
		# Pausing for `tsample` to give CPU time to process encoder signal
		#time.sleep(tsample)
		
		datar = ToF_read(sense,datar_old)
		datar_old[0] = datar[0]
		datar_old[1] = datar[1]
		datar_old[2] = datar[2]
		datar_old[3] = datar[3]
		# print("S{}: {}mm".format(1,datar[0]))
		# print("S{}: {}mm".format(2,datar[1]))
		
		print("S{}: {}mm S{}: {}mm S{}: {}mm S{}: {}mm".format(1,datar[0],2,datar[1],3,datar[2],4,datar[3]))
		
		#---
		
		# Getting current time (s)
		tcurr1 = time.perf_counter() - tstart
		dt1 = (tcurr1-tprev1)
		tprev1 = tcurr1
		#print (dt1)
		
		#---
		[Gx_w, Gy_w, Gz_w] = IMU.readGyro()
		Gx = Gx + Gx_w * 1.4*dt1
		Gy = Gy + Gy_w * 1.4*dt1
		Gz = Gz + Gz_w * 1.4*dt1

		#--
		#yaw = int(IMU.prevYaw[0])
		#print ("Yaw_mag: {}".format(yaw))
		#print(IMU.readGyro())
		print("Yaw: {}".format(int(Gz)))
		#print(IMU.prevAngle[0])
		
		#---
		state = button_callback(18)
		if (state != prev_state):
			print ("CHANGE")
			countCh = countCh + 1
			if (countCh == 2):
				if (stop == 0):
					f1 = 5
				if (stop == 1):
					f1 = 4
				countCh = 0
				
		prev_state = state
		
		STR_L = datar[0]
		STR_R = datar[1]
		S_R = datar[2]
		S_L = datar[3]
		
		
		
		if (f1 == 0) and (o < 20): 
			o = o + 1
		if ((STR_L < 150) or (STR_R < 150)) and (f1 == 0) and (o == 20):
			if (S_R <= S_L):
				f1 = 2
			if (S_L < S_R):
				f1 = 6
			t1 = int(Gz)
			#print("t1: {}".format(t1))
		if (f1 == 2):
			DEF1=5
			DEF2=5
			m = (int(Gz) - t1)
			print("Diff: {}".format(m))
			if (m > (90)):
				f1 = 3
				o1 = 0
				DEF1 = 0
				DEF2 = 0
		if (f1 == 6):
			DEF1=-5
			DEF2=-5
			m = (int(Gz) - t1)
			print("Diff: {}".format(m))
			if (m < (-90)):
				f1 = 3
				o1 = 0
				DEF1 = 0
				DEF2 = 0
		if (f1 == 3): #and ((int(Gz) - t1) < 90):
			o1 = o1 + 1
			if (o1 > 50):
				f1 = 4
		if (f1 == 4): #and ((int(Gz) - t1) < 90):
			f1 = 0
			o = 0
			countCh = 0
			stop = 0
			Gx = 0
			Gy = 0
			Gz = 0
			P = 1.0
			I = 10.0	#5.0
			D = 0	#0.001
			pid_R = PID(tsample, P, I, D, 100, -100, tau=taupid)
			pid_L = PID(tsample, P, I, D, 100, -100, tau=taupid)
			DEF1 = -10
			DEF2 = 10
		if (f1 == 5):
			DEF1 = 0
			DEF2 = 0
			stop = 1
		print("f1: {}".format(f1))
		#if (f1 == 1):
			#f1 = 0
		
		# Assigning motor output: I/O (data out)
		
		# if (i == 1):
		#anim_plot(j, wcurr_L, output_L, wcurr_R, output_R)
			# i = 0
		# fig.canvas.draw()
		# fig.canvas.flush_events()
		#time.sleep(1)
		#pass
	


def main():
	
	P = 1.0
	I = 10.0	#5.0
	D = 0	#0.001
	
	R = 0
	L = 0
	
	try:
		sense = ToF_init()
		detect_range(sense,count=5)
		
		pi = pigpio.pi()
		#button_init_pigpio(pi)

		pwm0,pwm1,pwm2,pwm3 = setup()
		
		button_init(18)
		
		#Setup the MinIMU_v5_pi
		IMU = MinIMU_v5_pi()
		#IMU.trackAngle()
		#Initiate tracking of Yaw on the IMU
		IMU.trackYaw()
		
# #------------------------		
		# pid_L = PID(P, I, D, setpoint=0)
		# #pid_L.SetPoint=0.0
		# pid_L.output_limits = (-100, 100)
		# #pid_L.setSampleTime(tsample)
		
		# pid_R = PID(P, I, D, setpoint=0)
		# #pid_R.SetPoint=0.0
		# pid_R.output_limits = (-100, 100)
		# #pid_R.setSampleTime(tsample)
# #------------------------		

		pid_R = PID(tsample, P, I, D, 100, -100, tau=taupid)
		pid_L = PID(tsample, P, I, D, 100, -100, tau=taupid)
		
		# calling the animation function
		#anim = animation.FuncAnimation(fig, animate, init_func = init, frames = None, interval = 20, blit = True)
		#plt.show()
		#anim.save('Rand.mp4', writer = 'ffmpeg', fps = 30) 
#------------------------

		decoder_R = rotary_encoder.decoder(pi, 5, 6, callback_R)
		decoder_L = rotary_encoder.decoder(pi, 12, 13, callback_L)
		
		runR(j,sense,IMU,pid_L,pid_R,pwm0,pwm1,pwm2,pwm3)

#------------------------		
		# # # # # #time.sleep(1)
		# num = mp.cpu_count()
		# pool = mp.Pool(num)
		
		# # # m4 = mp.Process(target=rotary_encoder.decoder, args = (pi, 5, 6, callback_L))
		# # # m3 = mp.Process(target=rotary_encoder.decoder, args = (pi, 12, 13, callback_R))
		# # # # m1 = mp.Process(target=callback_L, args = (way_L))
		# # # # m2 = mp.Process(target=callback_R, args = (way_R))
		# m1 = mp.Process(target=runR, args = (j,pid_L,pid_R,pwm0,pwm1,pwm2,pwm3))
		# # #m2 = mp.Process(target=anim_plot, args = (j, wcurr_L, output_L, wcurr_R, output_R))
		
		# # # m4.start()
		# # # m3.start()
		# # #m2.start()
		# m1.start()
		
		# # # # # #runR(pid_L,pid_R,pos_L,pos_R,pwm0,pwm1,pwm2,pwm3)
		
		# # # m4.join()
		# # # m3.join()
		# # #m2.join()
		# m1.join()
		
		# # # # # #runR(pid_L,pid_R,pos_L,pos_R,pwm0,pwm1,pwm2,pwm3)
#-------------------------

	#except Exception:
		#pass

	except KeyboardInterrupt:
		#print("Keyboard Interrupt")
		#pid_L = PID(0, 0, 0, setpoint=0)
		#pid_R = PID(0, 0, 0, setpoint=0)
		#plt.ioff()
		#runM(pwm0,pwm1,pwm2,pwm3,0,0,0,0)
		
		stopM(pwm0,pwm1,pwm2,pwm3)
		
		decoder_L.cancel()
		decoder_R.cancel()
		
		pi.stop()
		ToF_stop(sense)
		
		GPIO.cleanup()
	finally:
		#runM(pwm0,pwm1,pwm2,pwm3,DC0,DC1,DC2,DC3)
		stopM(pwm0,pwm1,pwm2,pwm3)
		decoder_L.cancel()
		decoder_R.cancel()
		pi.stop()
		#ToF_stop()
		#GPIO.cleanup()

#### Main ####
if __name__ == "__main__":
	main()
  
