import socket;
import sys;
import time;
import json

HOST,PORT="192.168.8.101",9999
data = {'command':'get-data'}

sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM);
sock.connect((HOST,PORT));

while True:
	print 'sent '+json.dumps(data);
       	sock.sendall(json.dumps(data));
	received = sock.recv(1024);
	print "recvied  {}".format(received);
	time.sleep(1);


