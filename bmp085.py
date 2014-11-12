
import Adafruit_BMP.BMP085 as BMP085;

import time
import threading
import datetime


class AverageWindow:
	def __init__(self,alpha):
		self.hasData=False;
		self.accumulator=0.0
		self.alpha=alpha
	def addValue(self,value):
		if(self.hasData == False):
			self.accumulator=value
			self.hasData=True
			print 'not has';
		else :
			self.accumulator = value*self.alpha + (1.0-self.alpha)*self.accumulator
	def getValue(self):
		return self.accumulator

class LowPassFilter:
	def __init__(self,alpha):
		self.hasData=False;
		self.lastValue=0.0
		self.alpha=alpha
	def addValue(self,value):
		if(self.hasData == False):
			self.lastValue=value;
			self.hasData=True;
		else :
			self.lastValue=self.lastValue + self.alpha*(value-self.lastValue);
	def getValue(self):
		return self.lastValue;

class RateLowPassFilter:
	def __init__(self,alpha):
		self.hasData=False;
		self.hasRate=False;
		self.lastValue=0.0
		self.alpha=alpha
		self.rate=0.0
	def addValue(self,value):
		if(self.hasData == False):
			self.lastValue=value;
			self.hasData=True;
			self.lastTime=datetime.datetime.now()
		else :
			end_time = datetime.datetime.now();
			delta_t = end_time-self.lastTime;
			dt = delta_t.microseconds/1.0E6 # assumes no delta_t is greater than 1s! 
			dv = value-self.lastValue
			new_rate = dv/dt
			
			if(self.hasRate == False):
				self.rate=new_rate 
				self.hasRate=True
			else:
				self.rate=self.rate + self.alpha*(new_rate-self.rate);

			self.lastValue=value;
			self.lastTime=end_time;

	def getValue(self):
		return self.rate;

class BMP085_Reader(threading.Thread) :
	def __init__(self):
		super(BMP085_Reader,self).__init__()
		self.sensor = BMP085.BMP085();
		self.readData=False;

		self.rawTemperature=0.0
		self.rawPressure=0.0
		self.rawAltitude=0.0

		self.lpfAltitude  = LowPassFilter(0.1)
		self.lpfTemperature= LowPassFilter(0.1)
		self.lpfPressure = LowPassFilter(0.01)

		self.setSeaLevelPressure=101964
		self.setAltitude=36

		self.lpfAltitudeRate = RateLowPassFilter(0.1)

	def run(self):
		while True:
			self.rawTemperature = self.sensor.read_temperature()
			self.rawPressure = self.sensor.read_sealevel_pressure(self.setAltitude)
			self.rawAltitude = self.sensor.read_altitude(self.setSeaLevelPressure)

			self.lpfAltitudeRate.addValue(self.rawAltitude)
			self.readData = True

			self.lpfAltitude.addValue(self.rawAltitude)
			self.lpfPressure.addValue(self.rawPressure)
			self.lpfTemperature.addValue(self.rawTemperature)

			time.sleep(0.1)

if __name__ == '__main__':
	bmp085= BMP085_Reader()
	bmp085.daemon=True
	bmp085.start()

	bmp085.setSeaLevelPressure=101896
	bmp085.setAltitude=36

	while True:
		params = dict(
			lpf_alt = bmp085.lpfAltitude.getValue(),
			lpf_temp= bmp085.lpfTemperature.getValue(),
			lpf_pressure= bmp085.lpfPressure.getValue(),
			lpf_altitudeRate = bmp085.lpfAltitudeRate.getValue())
		print str(params)

		time.sleep(1)




