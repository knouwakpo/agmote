import time
import datetime
import os

import json
import notecard
from periphery import I2C
import sys

sys.path.append('PATH-TO-THIS-FOLDER')
import sync_date_module as sync_date



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

sync_date.sync_time(card)
