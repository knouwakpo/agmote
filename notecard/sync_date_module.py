import time
import datetime
import os



def hub_sync(card):
    req = {"req": "hub.sync"}
    rsp = card.Transaction(req)
    print(rsp)
    time.sleep(0.5)
    while True:
        req = {"req": "hub.sync.status"}
        rsp = card.Transaction(req)
        print(rsp)
        if 'completed' in rsp:
            break
        time.sleep(0.5)

def sync_time(card):
    hub_sync(card)
    req = {"req": "card.time"}
    rsp = card.Transaction(req)
    #print(rsp)
    if 'time' in rsp:
        cur_epoch = rsp['time']
        #cur_date = datetime.datetime.fromtimestamp(rsp['time'])
        #print(cur_date)
        os.system(f"sudo date -s \'@{cur_epoch}\'")
        print(cur_epoch)
