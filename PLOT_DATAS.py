from tkinter import *
import tkinter.font
from PIL import Image, ImageTk
import serial
import time
import re
import sys,os
import platform
from random import randint
import serial,serial.tools.list_ports
import struct
import numpy as np
import matplotlib.pyplot as plt
import keyboard



#Init Var------------------
old_var = [0,0,0,0]
x=0
spend=0
X,Z,X_c,Z_c =0,0,0,0


#Find all Serial Ports activated and select only the port of the STM32 board----------------------------------------------------------------------
def find_USB_device():
    #serial.tools.list_ports -> trouve les periphériques connéctés
    myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]  

    #Look over all ports connected for "STM" port
    for p in myports:                                                       
        if "STM" in p[1]:                                                   # Select STM32 port COM 
            return p[0]                                                     #PORT FOUND
    return None;                                                            #PORT NOT FOUND


#RECEIVE AND TREAT DATAS FROM FRAME---------------------------------------------------------------------------------------------------------------
def read_datas(port_com):
    nb_octet = 17       #number of bytes in the frame
    checksum = 0        #checksum of frame
    timeout_UART = 0    #counter for timeout (UART)
    carac =''           #store bytes frome serial reading

    global old_var

    #Set serial parameters
    ser = serial.Serial(port_com,
                    baudrate=115200, 
                    bytesize=serial.EIGHTBITS, 
                    parity=serial.PARITY_NONE, 
                    stopbits=serial.STOPBITS_ONE, 
                    timeout=100) #Serial parameters

    #Wait for start character ($) until timeout
    while((not("$" in carac)) and (timeout_UART <=500)):         
        carac = str(ser.read())
        timeout_UART +=1

    #Check if timeout is exceed
    if(timeout_UART>=50):
        print("SERIAL COM NOT DETECTED")
        return 0, 0, 0, 0


    #Wait for entire byte frame
    while(ser.in_waiting <= nb_octet):
        pass 

    #Read next X bytes       
    s = ser.read(nb_octet)                       
    


    #Unpacked  bytes to original format (fff B)  f->float B->Byte
    int_vals = struct.unpack('ffffB', s)  

    #Unpacked each bytes in table to compute checksum  
    octets = struct.unpack('BBBBBBBBBBBBBBBBB', s)     

    #Compute checksum
    for i in octets[:-1]:
        checksum ^= i

    #Verif Checksum and extract interesting value 
    if(checksum == octets[-1]):                    #(Octets[-1] = last byte is the checksum)
        X= int_vals[1]
        Z= int_vals[0]
        X_c=int_vals[3]
        Z_c=int_vals[2]

        old_var[1]= X
        old_var[0]= Z
        old_var[3]= X_c
        old_var[2]= Z_c

    #In case of Value error print additionnal infos and return saved values
    else:
        print("- CHECKSUM ERROR :( -")
        X = old_var[1]
        Z = old_var[0]
        X_c = old_var[3]
        Z_c = old_var[2]
    return X,Z,X_c,Z_c


#Init graphs------------------------------------------------------------------------------------------------------------------------------------------
def Init_axis(ax1, ax2):
    X,Z,X_c,Z_c = read_datas(port_com)
    ax1.set_ylabel('X (°)',fontweight="bold",color ="red",size= 15)
    ax2.set_ylabel('Z (°)',fontweight="bold",color ="blue",size= 15)
    ax1.set_xlabel('Time')
    ax2.set_xlabel('Time')
    ax1.set(xlim=(0, 10))
    ax2.set(xlim=(0, 10))


#Update graphs datas-----------------------------------------------------------------------------------------------------------------------------------
def Plot_axis(ax1,ax2):
    ax1.plot(x, X,'.',color ="red")         #X axis measured
    ax1.plot(x, X_c,'k_')       #X axis setpoint
    ax2.plot(x, Z,'.', color= "blue")         #Z axis measured
    ax2.plot(x, Z_c,'k_')       #Z axis setpoint
    


#Compute and print real time datas IMU-----------------------------------------------------------------------------------------------------------------
def Run_graph():
    global x,spend,X,Z,X_c,Z_c
    #Loop for printing 
    while(keyboard.is_pressed("p") != True):
        start = time.time()         
        x=x+spend               #add spend time to abcisse value

        X,Z,X_c,Z_c = read_datas(port_com)      #read IMU datas and setpoints
        

        Plot_axis(ax1,ax2)      #plot X Y values of IMU datas + setpoints
        
        if(x>=10):              #Reset each 10 seconds
            ax1.clear()
            ax2.clear()
            Init_axis(ax1, ax2)
            x=0

        plt.pause(0.001)
        end = time.time()
        spend =end - start          #compute spend time




#------------------------------------------------------------------------------------------------------------------------------------------------------
# MAIN ------------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    #Connection to device
    port_com =  find_USB_device()

    #Create and init figure with two graphs 
    fig, (ax1, ax2) = plt.subplots(2,figsize=(14,7))        
    fig.suptitle('Real time IMU datas',size= 26,fontweight="bold")

    Init_axis(ax1, ax2)

    Run_graph()

    while(keyboard.is_pressed("esc") != True ):
        plt.pause(0.1)
        if(keyboard.is_pressed("r") == True ):
            Init_axis(ax1, ax2)
            Run_graph()
