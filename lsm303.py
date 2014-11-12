#from Adafruit_I2C import Adafruit_I2C
import Adafruit_GPIO.I2C as I2C
import numpy 
import math
import time
import datetime
import threading


def v3_scale(f, v):
	return [v[0]*f,v[1]*f,v[2]*f]
def v3_dot(a,b):
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def v3_normalize(v):
	mag = math.sqrt(v3_dot(v,v))
	return v3_scale(1.0/mag,v); 
def v3_add(a,b):
	return [a[0]+b[0],a[1]+b[1],a[2]+b[2]];
def v3_sub(a,b):
	return [a[0]-b[0],a[1]-b[1],a[2]-b[2]];
def v3_cross(u,v):
	x = u[1]*v[2] - u[2]*v[1]
	y = u[2]*v[0] - u[0]*v[2]
	z = u[0]*v[1] - u[1]*v[0]
	return [x,y,z];
def v3_average(a,b,c,w):
	x = a[0]*w[0]+b[0]*w[1]+c[0]*w[2]
	y = a[1]*w[0]+b[1]*w[1]+c[1]*w[2]
	z = a[2]*w[0]+b[2]*w[1]+c[2]*w[2]
	return [x,y,z]
	
class Phil_LSM303:
	LSM303_ADDRESS_MAG = 0x1E # compass.
	LSM303_REGISTER_MAG_CRA_REG_M = 0x00
	LSM303_REGISTER_MAG_CRB_REG_M = 0x01
	LSM303_REGISTER_MAG_MR_REG_M  = 0x02 # compass,MR_REG_M register.
	LSM303_REGISTER_MAG_OUT_X_H_M = 0x03 # output registers

	LSM303_ADDRESS_ACCEL = 0x19 # accelerometer.
	LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20
	LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23
	LSM303_REGISTER_ACCEL_OUT_X_L_A   = 0x28

	L3GD20H_ADRESS_GYRO = 0x6b # gyroscope
	L3GD20H_REGISTER_CTRL_1 = 0x20
	L3GD20H_REGISTER_CTRL_2 = 0x21
	L3GD20H_REGISTER_CTRL_3 = 0x22
	L3GD20H_REGISTER_CTRL_4 = 0x23
	L3GD20H_REGISTER_OUT_L_G = 0x28

	def __init__(self):
		self.compass = I2C.Device(self.LSM303_ADDRESS_MAG,1)
		self.compass.write8(self.LSM303_REGISTER_MAG_CRA_REG_M,0x14) # disable temperature, 30Hz output rate
		self.compass.write8(self.LSM303_REGISTER_MAG_MR_REG_M,0x00)  # put sensor into continuous mode from sleep mode.
		self.compass.write8(self.LSM303_REGISTER_MAG_CRB_REG_M,0x20) # set range +- 1.3 gauss, good enough for earth's mag field.


		self.accel = I2C.Device(self.LSM303_ADDRESS_ACCEL,1)
		self.accel.write8(self.LSM303_REGISTER_ACCEL_CTRL_REG1_A,0x27) # 10hz,low power disabled,x,y,z enabled.
		self.accel.write8(self.LSM303_REGISTER_ACCEL_CTRL_REG4_A,0x48) # BDU disable,big endian,+- 2G, low res (10bit)

		self.gyro = I2C.Device(self.L3GD20H_ADRESS_GYRO,1)
		self.gyro.write8(self.L3GD20H_REGISTER_CTRL_1,0x0F) # normal power, all axes enabled.
		self.gyro.write8(self.L3GD20H_REGISTER_CTRL_4,0x70) # continuous update,2000dps

		self.angle_x = 0.0
		self.angle_y = 0.0
		self.angle_z = 0.0

		self.xa=0
		self.ya=0
		self.za=0
		
		self.xg=0
		self.yg=0
		self.zg=0

		self.hasData=False

		self.read_data(0) # initial data read.

		
	def mag(self,a,b):
		n = (a<<8)|b;
		return float(numpy.int16(n));# if n < 32768 else n-65536 # 2's complement signed.

	def make_a(self,v):
		range=4.0*9.81
		max=32768
		zero=0
		s=v/65536
		s = s*range
		return s

	def make_gyro(self,v):
		if(math.fabs(v) < 25):
			v=0
		return 0.070*float(v)

	def read_data(self,dt):
		list = []
		for x in range(0,6):
			a = self.compass.readU8(self.LSM303_REGISTER_MAG_OUT_X_H_M+x)
			list.append(a);
		self.xm = self.mag(list[0],list[1])
		self.zm = self.mag(list[2],list[3]) # x,z,y! 
		self.ym = self.mag(list[4],list[5])
		heading = math.atan2(self.ym,self.xm)

		#print x_h_m,y_h_m,z_h_m,heading*180.0/3.1415;
		
		accel_list = []
		for x in range(0,6):
			v= self.accel.readU8(self.LSM303_REGISTER_ACCEL_OUT_X_L_A+x)
			accel_list.append(v);
		self.xa = float(numpy.int16(accel_list[0] << 8 | accel_list[1]))
		self.ya = float(numpy.int16(accel_list[2] << 8 | accel_list[3]))
		self.za = float(numpy.int16(accel_list[4] << 8 | accel_list[5]))

		gyro_list = []
		for x in range (0,6):
			g = self.gyro.readU8(self.L3GD20H_REGISTER_OUT_L_G+x)
			gyro_list.append(g)

		self.gyro_x_raw = numpy.int16(gyro_list[0] << 8 | gyro_list[1])
		self.gyro_y_raw = numpy.int16(gyro_list[2] << 8 | gyro_list[3])
		self.gyro_z_raw = numpy.int16(gyro_list[4] << 8 | gyro_list[5])


		#self.calculate_r(dt);
		self.calculate_dcm(dt)

	def calculate_dcm(self,dt):
		self.xag = self.make_a(self.xa)
		self.yag = self.make_a(self.ya)
		self.zag = self.make_a(self.za)
		
		self.gyro_x = self.make_gyro(self.gyro_x_raw)
		self.gyro_y = self.make_gyro(self.gyro_y_raw)
		self.gyro_z = self.make_gyro(self.gyro_z_raw)
		
		K1 = [-self.xag,-self.yag,-self.zag]
		I1 = [self.xm,self.ym,self.zm]
		J1 = v3_cross(K1,I1)

		K1 = v3_normalize(K1)		
		I1 = v3_normalize(I1)
		J1 = v3_normalize(J1)
	
		if not self.hasData:
			self.K1B = K1; self.J1B = J1; self.I1B = I1;
			self.hasData=True
			return

		K0 = self.K1B; I0 = self.I1B; J0 = self.J1B

		dThetaA = v3_cross(K0,v3_sub(K1,K0))
		dThetaW = [-math.radians(self.gyro_x)*dt,-math.radians(self.gyro_y)*dt,-math.radians(self.gyro_z)*dt]
		dThetaM = v3_cross(I0,v3_sub(I1,I0))		

		dThetaAvg = v3_average(dThetaA,dThetaW,dThetaM,[0.1,0.9,0.0])
		
		I1Bt = v3_add(I0 , v3_cross(dThetaAvg,I0))
		J1Bt = v3_add(J0 , v3_cross(dThetaAvg,J0))

		Err = v3_dot(I1Bt,J1Bt)/2
		
		I1Bn = v3_sub(I1Bt , v3_scale(Err , J1Bt))
		J1Bn = v3_sub(J1Bt , v3_scale(Err , I1Bt))
		K1Bn = v3_cross(I1Bn,J1Bn)

		self.I1B = v3_normalize(I1Bn)
		self.J1B = v3_normalize(J1Bn)
		self.K1B = v3_normalize(K1Bn)

		
	def calculate_r(self,dt):

		xag = self.make_a(self.xa)
		yag = self.make_a(self.ya)
		zag = self.make_a(self.za)
		
		mag = math.sqrt(xag*xag+yag*yag+zag*zag)
		rxacc = xag/mag; ryacc=yag/mag; rzacc=zag/mag;
		
		self.gyro_x = self.make_gyro(self.gyro_x_raw)
		self.gyro_y = self.make_gyro(self.gyro_y_raw)
		self.gyro_z = self.make_gyro(self.gyro_z_raw)
		
		#print '{0:10.4} {1:10.4} {2:10.4}'.format(self.gyro_x,self.gyro_y,self.gyro_z);
		
		if not self.hasData:
			self.restx = rxacc;self.resty=ryacc; self.restz=rzacc;
			self.hasData=True
			return
		
		angle_xz_last = math.atan2(self.restx,self.restz)
		angle_xz_new = angle_xz_last - math.radians(self.gyro_y)*dt

		angle_yz_last = math.atan2(self.resty,self.restz)
		angle_yz_new = angle_yz_last + math.radians(self.gyro_x)*dt

		angle_xy_last = math.atan2(self.resty,self.restx)
		angle_xy_new = angle_xy_last + math.radians(self.gyro_z)*dt
		
		cos_xz = math.cos(angle_xz_new)
		cos_yz = math.cos(angle_yz_new)
		sin_xz = math.sin(angle_xz_new)
		sin_yz = math.sin(angle_yz_new)
		tan_yz = math.tan(angle_yz_new)
		tan_xz = math.tan(angle_xz_new)

		rxgyro = sin_xz/math.sqrt(1+cos_xz*cos_xz*tan_yz*tan_yz)
		rygyro = sin_yz/math.sqrt(1+cos_yz*cos_yz*tan_xz*tan_xz)
		#rzgyro = math.copysign(1,self.restz)*math.sqrt(1-rxgyro*rxgyro-rygyro*rygyro) #cos_yz/math.sqrt(1+cos_yz*cos_yz*tan_xz*tan_xz)
		rzgyro = cos_yz/math.sqrt(1+cos_yz*cos_yz*tan_xz*tan_xz)

		#print self.rx,self.rz,angle_xz_last,angle_xz_new,math.radians(self.yg)*dt,r_new_x

		alpha = 0.98
		self.restx = alpha*rxgyro + (1-alpha)*rxacc
		self.resty = alpha*rygyro + (1-alpha)*ryacc
		self.restz = alpha*rzgyro + (1-alpha)*rzacc

		newmag = math.sqrt(self.restx*self.restx+self.resty*self.resty+self.restz*self.restz)

		self.restx = self.restx/newmag		
		self.resty = self.resty/newmag		
		self.restz = self.restz/newmag		

		#print '{0:0.2} {1:0.2} {2:0.2}'.format(self.restx,self.resty,self.restz);
		
class LSM_Reader(threading.Thread,Phil_LSM303) : 
		def __init__(self):
			threading.Thread.__init__(self)
			Phil_LSM303.__init__(self)
			self.stopRequested=False
		def run(self):
			dt = 0
			while not self.stopRequested:
				start = datetime.datetime.now()
				self.read_data(dt)
				time.sleep(0.001);
				end = datetime.datetime.now();			
				delta = end-start;
				dt=delta.microseconds/1.0E6

if __name__ == '__main__':

	import SocketServer

	lsm303 = LSM_Reader()

	class TCPHandler_temp(SocketServer.BaseRequestHandler):
		def handle(self):
			while True:
				message = self.request.recv(1024).strip()
				if not message: 
					break;
				#reply = '{0:0.2} {1:0.2} {2:0.2}  '.format(lsm303.restx,lsm303.resty,lsm303.restz);
				reply = '{0:0.2} {1:0.2} {2:0.2} '.format(lsm303.I1B[0],lsm303.I1B[1],lsm303.I1B[2]);
				reply += '{0:0.2} {1:0.2} {2:0.2} '.format(lsm303.J1B[0],lsm303.J1B[1],lsm303.J1B[2]);
				reply += '{0:0.2} {1:0.2} {2:0.2}  '.format(lsm303.K1B[0],lsm303.K1B[1],lsm303.K1B[2]);
				
				print message,reply
				self.request.sendall(reply)

	lsm303.daemon=True
	lsm303.start()

	server = SocketServer.TCPServer(("192.168.8.101",4000),TCPHandler_temp)
	server.serve_forever()

	raw_input('key to exit')
	lsm303.stopRequested=True
	lsm303.join()

