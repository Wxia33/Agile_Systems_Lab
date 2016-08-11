import smbus
import math
import time
import csv
import os
import RPi.GPIO as GPIO
import numpy as np
GPIO.setmode(GPIO.BCM)

#Addresses on MPU9250 board to read data
gyro_x_addr = 0x43
gyro_y_addr = 0x45
gyro_z_addr = 0x47
accel_x_addr = 0x3B
accel_y_addr = 0x3D
accel_z_addr = 0x3F

GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def read(addr): #Read from address
    high = bus.read_byte_data(address, addr)
    low = bus.read_byte_data(address, addr+1)
    val = (high << 8) + low
    return val

def read_data(addr): #Read data from sensors
    val = read(addr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def set_data(): #Function to convert from raw to real values
    gyrox = read_data(gyro_x_addr) / 131
    gyroy = read_data(gyro_y_addr) / 131
    gyroz = read_data(gyro_z_addr) / 131
    accelx = read_data(accel_x_addr) / 16384.0
    accely = read_data(accel_y_addr) / 16384.0
    accelz = read_data(accel_z_addr) / 16384.0
    return gyrox, gyroy, gyroz, accelx, accely, accelz

def my_callback(channel): #Callback for button press to trigger and save data
    global starttime
    global datafile
    global go
    tnow = time.time()-starttime #Trigger time
    #t1 = time.time()
    print(tnow)
    go = False #stops Pi from pulling data from MPU
    fn = datafile.name 
    datafile.flush() #make sure the data file is flushed before reading
    data = np.genfromtxt(fn, delimiter=',', skip_header=1, skip_footer=1,
                     names=['t','ax','ay','az','gx','gy','gz']) #read data file
    datafile.seek(0) 
    datafile.truncate() #wipes data file to start fresh
    data['t'] = data['t'] - tnow #traslate time according to trigger
    data2 = data[np.where(data['t'] >= -2)] #keep 2 seconds before trigger
    data2 = data2[data2['t'].argsort()] #sorts data according to increasing time
    i = 0
    while os.path.exists("imuData%03d.csv" % i):
        i += 1
    np.savetxt("imuData%03d.csv" % i, data2, delimiter=",") #save truncated data
    #print(time.time() - starttime)    
    print("File imuData%03d.csv saved" % i)
    go = True
    #t2 = time.time() - t1 #variable to measure elapsed time delay of thread
    #print(t2)
    

bus = smbus.SMBus(1)
address = 0x68
bus.write_byte_data(address, 0x6b, 0)
bus.write_byte_data(address, 0x1a, 1) 
bus.write_byte_data(address, 0x19, 1) #sample rate divider

bus.write_byte_data(address, 0x1b, 2) #Set Gyro Scale Range to 500 deg/s (32.8lsb/s)
bus.write_byte_data(address, 0x1c, 0) #Set Accel Scale Range
bus.write_byte_data(address, 0x1d, 3) #Accel Config2 Set 1KHz sampling rate

bus.write_byte_data(address, 0x37, 0x22)
bus.write_byte_data(address, 0x38, 0x01)


datafile = open("data.csv", 'wb')

# imuString = "Imu_Data" + date + ".csv"
# datafile = open(imuString,'wb')
writer = csv.writer(datafile)
writer.writerow(['time','accelx','accely','accelz',
    'gyrox','gyroy','gyroz'])

# CSV file title is formatted according 
# to: Year_Month_Day_Hour_Minute_Second

go = True
GPIO.add_event_detect(4, GPIO.FALLING, callback=my_callback, bouncetime=500)
count = 0
starttime = time.time()
while True: #Main Program Loop
    try:
        if go:
            t = time.time()-starttime #timestamp
            gx, gy, gz, ax, ay, az = set_data()
            writer.writerow([t,ax,ay,az,gx,gy,gz])
            count += count
            if count > 2000: #loop back to beginning of file to keep filesize down
                datafile.seek(0)
                count = 0
    except KeyboardInterrupt:
        endtime = time.time()
        print(endtime - starttime)
        GPIO.cleanup()
        datafile.close()
        break

