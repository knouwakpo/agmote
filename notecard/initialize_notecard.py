import time
import datetime
import os

import json
import notecard
from periphery import I2C
import sys

sys.path.append('PATH-TO-THIS-FOLDER')
import sync_date_module as sync_date



productUID = "YOUR-PRODUCTID-HERE"
port = I2C("/dev/i2c-1")
card = notecard.OpenI2C(port, 0, 0)

def set_productUID():
	req = {"req": "hub.set"}
	req["product"] = productUID
	req['mode'] = "periodic"
	req['inbound'] = 65
	req['outbound'] = 125
	print(json.dumps(req))
	rsp = card.Transaction(req)
	print(rsp)

req = {"req": "card.contact"}
req['name'] = "FIRST LAST"
req['org'] = "YOUR-ORG"
req['role'] = "YOUR-ROLE"
req['email'] = "YOUR-EMAIL"
rsp = card.Transaction(req)

print(rsp)


set_productUID()


sync_date.sync_time(card)

