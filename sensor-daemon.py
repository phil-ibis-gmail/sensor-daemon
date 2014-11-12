import SocketServer
import threading
import time
import json
import lsm303 as LSM303
import bmp085 as BMP085

#lsm303 = LSM303.LSM_Reader()
#lsm303.daemon=True
#lsm303.start()

bmp085= BMP085.BMP085_Reader()
bmp085.daemon=True
bmp085.start()

class TCPHandler(SocketServer.StreamRequestHandler):
	
	def get_lsm_303(self,request):
		params = dict(
			type='lsm-303',
			xa=lsm303.xa,
			ya=lsm303.ya,
			za=lsm303.za,

			xg=lsm303.xg,
			yg=lsm303.yg,
			zg=lsm303.zg,

			angle_x = lsm303.angle_x,
			angle_y = lsm303.angle_y,
			angle_z = lsm303.angle_z
			)
		return params;

	def get_bmp_085(self,request):
		params = dict( 
			type='bmp-085',
			timestamp=time.strftime("%c"),
			raw_temperature = bmp085.rawTemperature,
			raw_pressure = bmp085.rawPressure,
			raw_altitude = bmp085.rawAltitude,

			lpf_temperature= bmp085.lpfTemperature.getValue(),
			lpf_pressure = bmp085.lpfPressure.getValue(),
			lpf_altitude = bmp085.lpfAltitude.getValue(),

			lpf_altitude_rate = bmp085.lpfAltitudeRate.getValue(),
			
			set_altitude=bmp085.setAltitude,
			set_seaLevelPressure= bmp085.setSeaLevelPressure
			)
		return params;	
		

	def get_data(self,request):
		params = dict(bmp_085=self.get_bmp_085(request)); #,lsm_303=self.get_lsm_303(request));
		return params;

	def set_sea_level_pressure(self,arguments):
		print 'setting'+str(arguments);
		bmp085.setSeaLevelPressure=float(arguments['value']);
		return self.get_data(arguments)

	def parse_request(self,request):
		handlerTable = {'get-data': self.get_data, 'set-sea-level-pressure':self.set_sea_level_pressure}
		data = json.loads(request)
		answer = handlerTable[data['command']](data)
		return json.dumps(answer)

	def handle(self):
		while True:
			request = self.rfile.readline().strip()
			if not request: # socket closed on other end.
				break
			reply = self.parse_request(request)
			self.wfile.write(reply)

class ThreadedTCPServer(SocketServer.ThreadingMixIn,SocketServer.TCPServer): 
	pass

if __name__ == '__main__':
	HOST,PORT='192.168.8.101',9999;
	server = ThreadedTCPServer((HOST,PORT),TCPHandler);
	server.serve_forever();

