import matplotlib.animation as animation  
import matplotlib.pyplot as plt  
import numpy as np
import random  
import time  
  
x = []
y = [] 
z = []
y1 = [] 
z1 = []   

plt.ion()
# creating a blank window 
# for the animation  
fig = plt.figure() 
axis = plt.axes(xlim =(0, 100), 
                ylim =(-100, 100))  
  
line1, = axis.plot([], [], lw = 2, label = 'wcurr_L')  
line2, = axis.plot([], [], lw = 2, label = 'PIDout_L') 
line3, = axis.plot([], [], lw = 2, label = 'wcurr_R')  
line4, = axis.plot([], [], lw = 2, label = 'PIDout_R') 

axis.legend()
fig.suptitle('Motor_Control')
   
# what will our line dataset 
# contain? 
def init():  
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], []) 
    return line1, line2, line3, line4
   
# initializing empty values 
# for x and y co-ordinates 
xdata, ydata, zdata, ydata1, zdata1 = [], [], [], [], []

#rolling window size
repeat_length = 50

# animation function  
def anim_plot(i, a, b, a1, b1):  
    # t is a parameter which varies 
    # with the frame number 
    t = i  
       
    # x, y values to be plotted  
    x = t #* np.sin(t)  
    y = a #random.randint(1,10)#t * np.cos(t)
    z = b #random.randint(1,10)  
    y1 = a1 #random.randint(1,10)#t * np.cos(t)
    z1 = b1 #random.randint(1,10) 
       
    # appending values to the previously  
    # empty x and y data holders  
    xdata.append(x)  
    ydata.append(y)
    zdata.append(z) 
    ydata1.append(y1)
    zdata1.append(z1)
    
    line1.set_data(xdata, ydata)
    line2.set_data(xdata, zdata)
    line3.set_data(xdata, ydata1)
    line4.set_data(xdata, zdata1)
    
    if t>repeat_length:
        lim = axis.set_xlim(t-repeat_length, t)
    else:
        lim = axis.set_xlim(0,repeat_length)
    
    fig.canvas.draw()
    fig.canvas.flush_events()
    #plt.show(block=False)
    #time.sleep(0.01)  
    return line1, line2, line3, line4
   
# # animation function  
# def animate(i):  
    # # t is a parameter which varies 
    # # with the frame number 
    # t = i  
       
    # # x, y values to be plotted  
    # x = t #* np.sin(t)  
    # y = random.randint(1,10)#t * np.cos(t)  
       
    # # appending values to the previously  
    # # empty x and y data holders  
    # xdata.append(x)  
    # ydata.append(y)  
    # line.set_data(xdata, ydata)
    
    # if t>repeat_length:
        # lim = axis.set_xlim(t-repeat_length, t)
    # else:
        # lim = axis.set_xlim(0,repeat_length) 
      
    # return line, 

# # calling the animation function
# anim = animation.FuncAnimation(fig, animate, init_func = init, frames = None, interval = 20, blit = True)

# plt.show()
# #anim.save('Rand.mp4', writer = 'ffmpeg', fps = 30) 
