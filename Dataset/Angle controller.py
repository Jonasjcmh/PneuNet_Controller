import numpy as np
import pylab
from scipy.optimize import curve_fit
import pandas as pd
from scipy import signal
import csv

import pandas as pd
from scipy import signal
import csv

from math import*

import time
import serial         #conda install -c anaconda pyserial
import numpy as np    #conda install -c anaconda numpy
import numpy.linalg as la
from datetime import datetime
import math
import json          #conda install -c jmcmurray json  
from scipy import stats  #conda install -c anaconda scipy
import matplotlib.pyplot as plt  #conda install -c conda-forge matplotlib

from IPython.display import clear_output
from IPython.display import Image




def star_communication(port):
    ser = serial.Serial(port, 115200, timeout=1)   #Include the number of the serial port
    return ser
    
def close_communication(ser):
    motor_control(ser,1)
    ser.close()


def read_data(ser, degree):
    #ser_bytes = ser.readline()
    ser.flushInput()
    
    cmd='r'+str(degree)+'\n'
    ser.write(bytes(cmd, encoding='ascii'))
    
    time.sleep(0.015)
    ser_bytes = ser.readline(ser.inWaiting())
    decoded_bytes = ser_bytes.decode()
    if len(decoded_bytes)>0 :
        decoded_bytes=decoded_bytes.strip()
        separated_data=decoded_bytes.split(",")
        if len(separated_data[0])>=4:
            integer_data = [i for i in separated_data]
            integer_array= np.array(integer_data)
        else:
            integer_array=np.zeros(5)
            integer_array[0]=2000
    else:
        integer_array=np.zeros(5)
        integer_array[0]=2000
    return integer_array

    
def feedback_data(ser, degree):              #current angle, current pressure, set_point(pressure or angle controller depends of the control), motor_speed
    integer_array=read_data(ser,degree)
    while len(integer_array)<5 or integer_array[0]==2000:
        integer_array=read_data(ser, degree)
    integer_array2=integer_array
    return integer_array2

def feedback_data2(ser, degree):              #current angle, current pressure, set_point(pressure or angle controller depends of the control), motor_speed
    integer_array=read_data(ser,degree)
    integer_array2=integer_array
    return integer_array2



def motor_control(ser, speed_val):    #speed on the range of +255 to -255, Negative values just open the valve and stop the pump, 0 stops the pump but valve is still working
    ser_bytes = ser.readline()
    ser.flushInput()
    time.sleep(0.001)
    motor_speed=str(speed_val)
    cmd='m'+motor_speed+'\n'
    print(cmd)
    ser.write(bytes(cmd, encoding='ascii'))

def pressure_control(ser, pressure_val):    #pressure on the range of 0 to +25 kpa
    ser_bytes = ser.readline()
    ser.flushInput()
    time.sleep(0.001)
    motor_pressure=str(pressure_val)
    cmd='p'+motor_pressure+'\n'
    #print(cmd)
    ser.write(bytes(cmd, encoding='ascii'))
    
def bend_control(ser, bend_val):    #angle on the range of 0 to +180
    ser_bytes = ser.readline()
    ser.flushInput()
    time.sleep(0.001)
    motor_bend=str(bend_val)
    cmd='a'+motor_bend+'\n'
    #print(cmd)
    ser.write(bytes(cmd, encoding='ascii'))
    
def calibrate_zero(ser):    #speed on the range of +255 to -255, Negative values just open the valve and stop the pump, 0 stops the pump but valve is still working
    ser_bytes = ser.readline()
    ser.flushInput()
    time.sleep(0.001)
    cmd='l'+'0'+'\n'
    print(cmd)
    ser.write(bytes(cmd, encoding='ascii'))
    
    
    

Fs=16000
f=280
sample=120
a=[0]*sample
for n in range(sample):
    a[n]=int(60+60*sin(3*pi/2-2*pi*f*n/Fs))
plt.plot(a)

i=0
j=0
k=0
name="Dataset/Sin_dataset_11.csv"

max_value=120
min_value=0

steps=1
samples=120*steps


ser=star_communication('COM14')
time.sleep(5)
calibrate_zero(ser)
time.sleep(0.02)
calibrate_zero(ser)

bend_control(ser, 0)
values=feedback_data(ser,0)
to=time.perf_counter()
print('start')
with open(name,"a") as f:
            writer = csv.writer(f,delimiter=";",lineterminator='\r')
            writer.writerow(['time','angle','pressure','set_point','motor_speed'])

degree=min_value
while (i<samples):                      # Close loop controller
    degree=a[k]
    k=k+1
    if degree > max_value:
        degree=max_value;
    bend_control(ser, degree)
    
    time.sleep(0.002)
    
    
    #value2 = feedback_data2(ser,degree)
    #if len(value2)<5 or value2[0]==2000:
    #    values = value2
    
    values=feedback_data(ser,degree)
    with open(name,"a") as f:
                        writer = csv.writer(f,delimiter=";",lineterminator='\r')
                        writer.writerow([time.perf_counter()-to,values[0],values[1],values[2], values[3]])
    print(str(i)+' '+str(values))
    i=i+1


close_communication(ser)

ser=star_communication('COM14')
time.sleep(2)
motor_control(ser,-10)
time.sleep(2)
ser.close()

with open(".csv","a") as f:
    df = pd.read_csv(name,delimiter=';')
    #Filtering
    b, a = signal.butter(3, 0.05)
    filter_angle = signal.filtfilt(b, a, df['angle'])
    df = df.assign(Filtered_angle=filter_angle)
    f.close()

df.to_csv("Dataset/Sin_dataset_11_filtered.csv")
    
a=2*samples
ydata=df['Filtered_angle']
ydata2=df['set_point']
ydata3=df['Filtered_angle']
xdata=df['time']
ydata=ydata[1:a]
ydata2=ydata2[1:a]
ydata3=ydata3[1:a]
xdata=xdata[1:a]
fig = plt.figure(figsize=(10,5))
plt.plot(xdata, ydata2, label='Set point')
plt.plot(xdata, ydata, label='Sensed angle')
plt.xlabel('Time [s]')
plt.ylabel('Bending angle [degree]')
plt.title('Bending angle response to a sine reference signal')
plt.draw()
plt.grid(True)
plt.legend() 
fig.savefig('Sin_dataset_9.jpg', bbox_inches='tight', format='jpg')
