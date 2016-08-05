import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

def dist(a,b):
    return math.sqrt((a*a) + (b*b))

def rotX(x,y,z): #Roll
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def rotY(x,y,z): #Pitch
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def compFilter(gyrox, gyroy, gyroz, accelx, accely, accelz, dt):
    #Complementary Filter, combines accelerometer and gyroscope data
    Xangle = 0.98 * (rotX(accelx,accely,accelz) + gyrox * dt) + 0.02 * accelx
    Yangle = 0.98 * (rotY(accelx,accely,accelz) + gyroy * dt) + 0.02 * accely
    Zangle = math.degrees(gyroz * dt)
    return Xangle, Yangle, Zangle

def remGrav(ax, ay, az, alpha):
	# dax = []
	# day = []
	# daz = []
	# for i in range(len(ax) - 1):	
	# 	dax.append(ax[i+1] - ax[i])
	# 	day.append(ay[i+1] - ay[i])
	# 	daz.append(az[i+1] - az[i])
	gravx = 0
	gravy = 0
	gravz = 0
	linAccelx = []
	linAccely = []
	linAccelz = []
	for i in range(len(ax) - 1):
		gravx = alpha * gravx + (1 - alpha) * ax[i]
		gravy = alpha * gravy + (1 - alpha) * ay[i]
		gravz = alpha * gravz + (1 - alpha) * az[i]

		linAccelx.append(ax[i] - gravx)
		linAccely.append(ay[i] - gravy)
		linAccelz.append(az[i] - gravz)
		
	return linAccelx, linAccely, linAccelz

filename = 'imuData000TEST.csv' #open data file
data = np.genfromtxt(filename, delimiter=',', skip_header=1, skip_footer=1,
                     names=['t','ax','ay','az','gx','gy','gz']) #Label data
fig = plt.figure()
ax1 = fig.add_subplot(211) #generate subplot
ax2 = fig.add_subplot(212) #generate subplot
#ax3 = fig.add_subplot(413) #generate subplot
#ax4 = fig.add_subplot(414) #generate subplot

linAccelx, linAccely, linAccelz = remGrav(data['ax'],
                                          data['ay'],data['az'], 0.1)

linAccelx.append(0)
linAccely.append(0)
linAccelz.append(0)

angleX = []
angleY = []
angleZ = []
for i in range(0,len(data['ax'])):
    pitch, roll, yaw = compFilter(data['gx'][i], data['gy'][i],
                            data['gz'][i], data['ax'][i],
                            data['ay'][i], data['az'][i], 0.002)
    angleX.append(pitch)
    angleY.append(roll)
    angleZ.append(yaw)

t = range(0,len(linAccelx))


class KalmanFilter(object):

    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate
unfilpitch = []
unfilroll = []
unfilyaw = []
for i in range(0,len(data['ax'])):
    pitch = rotX(data['ax'][i],data['ay'][i],data['ax'][i]) + data['gx'][i] * .002
    roll = rotY(data['ax'][i],data['ay'][i],data['az'][i]) + data['gy'][i] * .002
    yaw = math.degrees(data['gz'][i] * .002)
    unfilpitch.append(pitch)
    unfilroll.append(roll)
    unfilyaw.append(yaw)

# in practice we would take our sensor, log some readings and get the
# standard deviation
measurement_standard_deviationX = np.std(angleX)
measurement_standard_deviationY = np.std(angleY)
measurement_standard_deviationZ = np.std(angleZ)

# The smaller this number, the fewer fluctuations, but can also venture off
# course...
process_variance = 10
estimated_measurement_varianceX = measurement_standard_deviationX ** 2
estimated_measurement_varianceY = measurement_standard_deviationY ** 2
estimated_measurement_varianceZ = measurement_standard_deviationZ ** 2
kalman_filterX = KalmanFilter(process_variance, estimated_measurement_varianceX)
kalman_filterY = KalmanFilter(process_variance, estimated_measurement_varianceY)
kalman_filterZ = KalmanFilter(process_variance, estimated_measurement_varianceZ)

posteri_estimate_graphX = []
posteri_estimate_graphY = []
posteri_estimate_graphZ = []

for iteration in xrange(0, len(data['ax'])):
    kalman_filterX.input_latest_noisy_measurement(angleX[iteration])
    posteri_estimate_graphX.append(kalman_filterX.
    	get_latest_estimated_measurement())
    kalman_filterY.input_latest_noisy_measurement(angleY[iteration])
    posteri_estimate_graphY.append(kalman_filterY.
    	get_latest_estimated_measurement())
    kalman_filterZ.input_latest_noisy_measurement(angleZ[iteration])
    posteri_estimate_graphZ.append(kalman_filterZ.
    	get_latest_estimated_measurement())

ax1.plot(data['t'], data['ax'], data['t'],
         data['ay'], data['t'], data['az']) #plot accelerometer data
ax2.plot(data['t'], data['gx'], data['t'],
         data['gy'], data['t'], data['gz']) #plot gyroscope data
ax3.plot(data['t'], data['mx'], data['t'],
         data['my'], data['t'], data['mz'])
#ax1.plot(data['t'], linAccelx, data['t'], linAccely, data['t'], linAccelz)
#ax1.plot(data['t'], angleX, data['t'], angleY, data['t'], angleZ)

#ax2.plot(data['t'], posteri_estimate_graphX,
#         data['t'], posteri_estimate_graphY,
#         data['t'], posteri_estimate_graphZ)
#         data['t'], angleX,
#         data['t'], angleY,
#         data['t'], angleZ) # plot filtered angle

plt.show() #show plots

