import time
import threading
import datetime
import gps
import json

class GPSReader(threading.Thread):
	def __init__(self):
		super(GPSReader,self).__init__()
		self.session = gps.gps("localhost","2947")
		self.session.stream(gps.WATCH_ENABLE|gps.WATCH_NEWSTYLE)
		self.hasDataFlag=False
	def run(self):
		while True:
			report = self.session.next()
			self.last_report = self.stupidDict(report)
			self.hasDataFlag = True
	def hasData(self):
		return self.hasDataFlag
	def consumeData(self):
		ret = self.last_report;
		self.hasDataFlag = False
		return ret
	# the gps library returns a 'dictwrapper' which is not a dict, and doesn't json serialize.
	def stupidDict(self,x):
		for key in x.keys() :
			if(type(x[key]) == list):
				for i,v in enumerate(x[key]):
					if(x[key][i].__class__ == gps.client.dictwrapper):
						x[key][i] = self.stupidDict(x[key][i])
			if(x[key].__class__ == gps.client.dictwrapper):
				x[key] = self.stupidDict(x[key])
		return x.__dict__;
		
if __name__ == '__main__':
	gpsReader = GPSReader()
	gpsReader.daemon=True;
	gpsReader.start()
	t = {'a':'b','c':'d'}
	while True:
		if(gpsReader.hasData()):
			data = gpsReader.consumeData()
			print data
			print t
			print json.dumps(data)
