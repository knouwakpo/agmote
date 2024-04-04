import time
import datetime
import os
import json
import threading
import sys
import glob


## May need to update clock using notehub before proceeding



def open_file():
    thistime = datetime.datetime.now()
    fname = thistime.strftime('%Y_%m_%d.txt')
    path_=os.path.join("/home/roccflume/Flume",fname)
    if os.path.exists(path_):
        fh = open(path_, 'a')
    else:
        fh = open(path_, 'w')
        fh.write("DateTime,Sender,Receiver,Rssi,data\n")
    return fh

def parse_line(line):
	line = line.strip('\n')
	arr = line.split(',')
	arr_dict = {}
	for pair_ in arr:
		pair_split = pair_.split(":")
		if len(pair_split) < 2:
			arr_dict['data'] = pair_split[0]
			continue
		if 'rcvd' in pair_split[0]:
			#arr_dict[pair_split[0]] = datetime.datetime.fromtimestamp(int(pair_split[1]))
			arr_dict[pair_split[0]] = int(pair_split[1])
		elif 'data' in pair_split[0]:
			arr_dict[pair_split[0]] = pair_split[1] ##TODO parse the data field in more details
		else:
			arr_dict[pair_split[0]] = eval(pair_split[1])
		
	return arr_dict
		
	

#Find temp files
temp_files = glob.glob("/home/roccflume/.apps/radiodata.temp.*.txt")
temp_files.sort()
print(temp_files)

#Get new packets and
#Consolidate / append temp files into the daily output

NEWPACKETS = []

with open_file() as fh:
	for file_ in temp_files:
		print(file_)
		with open(file_, "r") as fh_in:
			while(True):
				try:
					line = fh_in.readline()
				except:
					continue
				if line:
					NEWPACKETS.append(parse_line(line))
					fh.write(line)
				else:
					break


print(NEWPACKETS)				


## Now do the notehub stuff here

import notecard
from periphery import I2C
import threading
import sys

productUID = "YOUR-PRODUCTID"
port = I2C("/dev/i2c-1")
card = notecard.OpenI2C(port, 0, 0)



def set_productUID():
	req = {"req": "hub.set"}
	req["product"] = productUID
	req["mode"] = "continuous"

	print(json.dumps(req))

	rsp = card.Transaction(req)
	print(rsp)

req = {"req": "hub.get"}
rsp = card.Transaction(req)


if rsp['product'] == productUID:
	print('ProductUID already set')
else:
	set_productUID()
	#pass

MAX_PACKET_PER_NOTE = 120

def send_data_to_hub():
	global NEWPACKETS
	global card
	cur_buffer = NEWPACKETS.copy()
	try:
		print('Got Send Flag for', NEWPACKETS)
		if True: ##Delete before deploying
			notes = []
			req_body = {}
			if cur_buffer:
				if len(cur_buffer) > MAX_PACKET_PER_NOTE:
					sync_note = True
				else:
					sync_note = False
				for idata_, data_ in enumerate(cur_buffer):
					req_body[f'data{idata_:04d}'] = data_
					if ((idata_ >= MAX_PACKET_PER_NOTE) and ((idata_ % MAX_PACKET_PER_NOTE) == 0)):
						print(req_body)
						req = {"req": "note.add"}
						req["file"] = "flume.qo"#Change this
						req["sync"] = sync_note
						req["body"] = req_body
						rsp = card.Transaction(req)
						print(rsp)
						req_body = {}
				
				print(req_body)
				req = {"req": "note.add"}
				req["file"] = "flume.qo"#Change this
				req["sync"] = sync_note
				req["body"] = req_body
				rsp = card.Transaction(req)
				print(rsp)
	except:
		print('An error occured')

	finally:
		pass

if True:
	send_data_to_hub()

#delete temp files 
if True:
	for file_ in temp_files:
		os.remove(file_)
		











