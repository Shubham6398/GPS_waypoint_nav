import serial
import math
import time

a = [1500,0]
gfl = 0
imufl = 0
ardfl = 0
avel = 4
vel = 2.5
t = 0 
anger_p = 0
dist_p = 0
kp=6
ki=1.5
kd=0.5
#sergps = serial.Serial(port = '/dev/ttyUSB2',baudrate= 38400)
serthr = serial.Serial(port = '/dev/ttyACM0',baudrate= 115200)
#serimu = serial.Serial(port = '/dev/ttyUSB1',baudrate= 115200)

def send_PWM(ser,PWM1,PWM2):
	serthr = serial.Serial(port = '/dev/ttyACM0',baudrate= 115200)
	try:
		ardfl = 0
        	to_send = str(PWM1) + ":" + str(PWM2) + ":|"
        	serthr.write(to_send.encode())
        	serthr.flushInput()
	except:
		ardfl = 1
		pass

def imu():
	serimu = serial.Serial(port = '/dev/ttyACM0',baudrate= 115200)
	#command = ("$PSRFS,yaw,get,GLOM,RPT=0.3")
	#serimu.write(command + '\r'+'\n')
	serimu.flush()
	while True:
		data = serimu.readline()
		for line in data.split('\n'):
			if line.startswith('$'):
				head = line.split(',')
				print str(head[3]) + "|" + str(head[4])
				head = float(head[1])
				print head
				serimu.flush()
				return head

def gps():
	sergps = serial.Serial(port = '/dev/ttyUSB0',baudrate= 38400)
	gfl = 0
	while gfl == 0:
		try:
			data = sergps.readline()
			for line in data.split('\n') :
				if line.startswith( '$GPGGA' ) :
					lat, _, lon = line.strip().split(',')[2:5]
					gfl = 1
		except:
			pass
		try:
			lat = float(lat)
			lon = float(lon)
			cord = [lat/100,lon/100]
		except:
			pass
	return cord


def head_calc(dest):

	cur = gps()
	x = dest [0] - cur[0]
	y = dest [1] - cur[1]
	d = dist(dest)
	sin = math.degrees(math.asin(abs(y/d)))
	
	if ( x > 0 and y > 0 ):
		#Quadrant 1
		ang = sin
	if ( x < 0 and y > 0 ):
		#Quadrant 2
		ang = 180 - sin
	if ( x < 0 and y < 0 ):
		#Quadrant 3
		ang = 180 + sin
	if ( x > 0 and y < 0 ):
		#Quadrant 4
		ang = 360 - sin
	return ang

def dist(y):

	r = 6371000
	x = gps()
	p1 = math.radians(x[0])
	p2 = math.radians(y[0])
	w = math.radians(y[0]-x[0])
	l = math.radians(y[1]-x[1])
	a = (math.sin(w/2)*math.sin(w/2))+(math.cos(p1)*math.cos(p2)*math.sin(l/2)*math.sin(l/2))
	c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a))
	d = r*c
	if (y[0]-x[0]) < 0 :
		d = d*(-1)
	return d

def drive(dest):

	global t
	global anger_p,dist_p
	global kp,ki,kd
	t = time.time()
	x = dist(dest)
	z = gps()
	head = imu()
	ang = head_calc(dest)
	dt = (time.time() - t)*1000
	ps = 1500
	pd = 0
	ianger = 0
	anger = abs(ang - head) - 180
	d_x = x - dist_p
	d_anger = x - anger_p
	#ianger += anger*dt 
	print str(ps) + '|' + str(x) + '|' +str(dt)+ '|' + str(d_x/dt)
	print str(pd) + '|' + str(ang) + '|' + str(head) + '|Anger' + str(anger) + '|' + str(dt) + '|' + str(d_anger/dt) + '|' + str(ianger)
	ps = float (ps + kp*(x)*(0.5) + kd*(d_x/dt))
	pd = float (pd + kp*(anger)*(2.2) + kd*(d_anger/dt) )
	ps = int(abs(ps))
	pd = int(pd)
	a = [ ps , pd ]
	return a

#waypoint = [[12.824194,80.040732],[12.824186,80.040523],[12.823822,80.040499],[12.823742,80.040744],gps()]
waypoint = [ [12.819072,80.041910] , [12.819030,80.041727] , [12.818953,80.041895] , gps()]
for i in (0,10000):
	send_PWM (serthr, 1500 , 1500)
g = gps()
#g=[12.824186,80.040523]
print g
head = imu()
print head
pt = 0
if (waypoint[pt][0]-g[0] <= 0.000004) & (waypoint[pt][1]-g[1] <= 0.000004):
    	pt += 1
while True:
	try:
		if (waypoint[pt][0]-g[0] <= 0.000004) & (waypoint[pt][1]-g[1] <= 0.000004):
				pt += 1
		a = drive (waypoint[pt])
		a[0] = 1500
		send_PWM(serthr,a[0]-(a[1]/2),a[0]+(a[1]/2))
		lt = a[0]+(a[1]/2)
		rt = a[0]-(a[1]/2)
		print "Left PWM:" + str(lt) + '|' + "RightPWM:" + str(rt) + '|' + "Linear PWM:" +  str(a[0]) + '|' + "Differential PWM:" + str(a[1]) + '\n'
	except KeyboardInterrupt:
		for i in (0,100000):
			send_PWM(serthr,1500,1500)
		print('exit')
		break
