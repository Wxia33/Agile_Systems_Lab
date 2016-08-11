import smbus
import math
import time
import csv
import os
import numpy as np

#Addresses on MPU9250 board to read data
gyro_x_addr = 0x43
gyro_y_addr = 0x45
gyro_z_addr = 0x47
accel_x_addr = 0x3B
accel_y_addr = 0x3D
accel_z_addr = 0x3F

mag_address = 0x0c

mag_x_addr_l = 0x03
mag_x_addr_h = 0x04
mag_y_addr_l = 0x05
mag_y_addr_h = 0x06
mag_z_addr_l = 0x07
mag_z_addr_h = 0x08

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

def us2s(usd): #for magnetometer reading
    if usd & (0x01 << 15) : 
        tmp = usd ^ 0xffff
        tmp = tmp + 1
        sd = -1 * tmp
    else:
        sd=usd
    return sd

def getMag(axis):
    if axis == 'X':
        mh=bus.read_byte_data(mag_address, mag_x_addr_h)    
        ml=bus.read_byte_data(mag_address, mag_x_addr_l)    

    if axis == 'Y':
        mh=bus.read_byte_data(mag_address, mag_y_addr_h)    
        ml=bus.read_byte_data(mag_address, mag_y_addr_l)    
 
    if axis == 'Z':
        mh=bus.read_byte_data(mag_address, mag_z_addr_h)    
        ml=bus.read_byte_data(mag_address, mag_z_addr_l)    

    bus.read_byte_data(mag_address, reg_mag_st)
    usm = mh << 8 | ml
    sm = us2s(usm)
    mag = sm * 0.15
    return mag

def set_data(): #Function to convert from raw to real values
    gyrox = read_data(gyro_x_addr) / 131
    gyroy = read_data(gyro_y_addr) / 131
    gyroz = read_data(gyro_z_addr) / 131
    accelx = read_data(accel_x_addr) / 16384.0
    accely = read_data(accel_y_addr) / 16384.0
    accelz = read_data(accel_z_addr) / 16384.0
    return gyrox, gyroy, gyroz, accelx, accely, accelz

def my_callback(): #Callback for button press to trigger and save data
    global starttime
    global datafile
    tnow = time.time()-starttime #Trigger time
    #t1 = time.time()
    print(tnow)
    fn = datafile.name 
    datafile.flush() #make sure the data file is flushed before reading
    data = np.genfromtxt(fn, delimiter=',', skip_header=1, skip_footer=1,
                     names=['t','ax','ay','az','gx','gy','gz','mx','my','mz']) #read data file
    datafile.seek(0) 
    datafile.truncate() #wipes data file to start fresh
    data['t'] = data['t'] - tnow #traslate time according to trigger
    data2 = data[np.where(data['t'] >= -2)] #keep 2 seconds before trigger
    data2 = data2[data2['t'].argsort()] #sorts data according to increasing time
    i = 0
    while os.path.exists("imuData%03dTEST.csv" % i):
        i += 1
    np.savetxt("imuData%03dTEST.csv" % i, data2, delimiter=",") #save truncated data
    #print(time.time() - starttime)    
    print("File imuData%03dTEST.csv saved" % i)
    #t2 = time.time() - t1 #variable to measure elapsed time delay of thread
    #print(t2)
    

bus = smbus.SMBus(1)
address = 0x68
bus.write_byte_data(address, 0x6b, 0)
bus.write_byte_data(address, 0x1a, 1) 
bus.write_byte_data(address, 0x19, 0) #Sets sampling rate divider

#bus.write_byte_data(mag_address, 0x0A, 0x1F)    # set mag-16bit , ROM mode
#magasaX = bus.read_byte_data(mag_address, 0x10) # read mag_asax
#magasaY = bus.read_byte_data(mag_address, 0x11) # read mag_asay
#magasaZ = bus.read_byte_data(mag_address, 0x12) # read mag_asaz

#bus.write_byte_data(mag_address, 0x0A, 0x10)    # set mag-16bit ,  powerdown mode
time.sleep(0.001)       # waiting

#bus.write_byte_data(mag_address, 0x0A, 0x16)    # set mag-16bit , mag-100hz

reg_mag_st = 9

datafile = open("data.csv", 'wb')

# imuString = "Imu_Data" + date + ".csv"
# datafile = open(imuString,'wb')
writer = csv.writer(datafile)
writer.writerow(['time','accelx','accely','accelz',
    'gyrox','gyroy','gyroz','magx','magy','magz'])

# CSV file title is formatted according 
# to: Year_Month_Day_Hour_Minute_Second

mx = 0
my = 0
mz = 0

count = 0
starttime = time.time()
t = time.time()-starttime #timestamp
while t < 3: #Main Program Loop
    try:
        t = time.time()-starttime #timestamp
        gx, gy, gz, ax, ay, az = set_data()
#        mxh=bus.read_byte_data(mag_address, mag_x_addr_h)
#        mxl=bus.read_byte_data(mag_address, mag_x_addr_l)
#        bus.read_byte_data(mag_address, reg_mag_st)
#        mx = mxh << 8 | mxl
#        mx = getMag("X") #Magnetometer data reading
#        my = getMag("Y")
#        mz = getMag("Z")
        writer.writerow([t,ax,ay,az,gx,gy,gz,mx,my,mz])
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

my_callback()