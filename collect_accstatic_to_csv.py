#WIP data collection for static calibration of body accelerometer
#Read data from Arduino serial monitor, convert to .csv file
#Values recorded: calibrated accelerometer values, motor feedback (accelerometer and velocity), potentiometer
#Values recorded: raw acceleromter readings for body accelerometer
#Use with collect_accstatic.ino

#Author: Alex Boehm
#created spring 2022 for WIP senior design

#How To Use:
#1. Plug in arduino and upload collect_accstatic.ino
#2. Open the serial monitor and check it's reading values from each sensor
#   The arduino should print the readings to serial monitor in the format: 
#       acc_cart_x,acc_cart_y,acc_cart_z,\n
#2. Close the serial monitor
#3. Update the port variable to the name of the COM port with the Arduino connection (line 96)
#   Update the name of the output file (line 97)
#4. Run python script (takes 30-40 seconds to run). File saves to same folder script is in.

import serial
import time
import matplotlib.pyplot as plt
import csv

class ADXL:
    def __init__(self, name, measurement_time, samplerate):
        self.name = name
        self.time = measurement_time
        self.rate = samplerate
        self.acc_cart_x = []
        self.acc_cart_y = []
        self.acc_cart_z = []

    def read_line(self,ser):
        #read one line from serial monitor
        line = ser.readline()
        if line:
            string = line.decode(encoding='UTF-8',errors='ignore')  #converts byte string into unicode
            # print(string)
            data = string.split(',')
            # print(data)
            if len(data) == 4:
                self.acc_cart_x.append(data[0])
                self.acc_cart_y.append(data[1])
                self.acc_cart_z.append(data[2])

    def take_reading(self,port):

        # connect to serial port for arduino
        ser = serial.Serial(port,9600,timeout=1)
        time.sleep(2)   #delays output to make sure connection is established

        #read serial port at sample rate
        num_samples = self.time*self.rate
        for sample in range(num_samples):
            self.read_line(ser)
            time.sleep(1/self.rate)
        
        #close the serial port connection
        ser.close()

    def export_to_csv(self,filename):
        with open(filename, mode='w',newline='') as file:
            file_writer = csv.writer(file)
            
            # write header
            file_writer.writerow(['acc_cart_x (raw)','acc_cart_y (raw)','acc_cart_z (raw)'])  
            
            #write each line of recorded data
            for line in range(len(self.pot)):
                row = [self.acc_cart_x[line],self.acc_cart_y[line],self.acc_cart_z[line]]
                file_writer.writerow(row)


measurement_time = 30   #s
samplerate = 10         #Hz
port = 'COM4'
filename = 'static_x.csv'

Acc = ADXL('Acc',measurement_time,samplerate)
Acc.take_reading(port)
Acc.export_to_csv(filename)
