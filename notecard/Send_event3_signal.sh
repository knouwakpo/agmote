#!/usr/bin/bash

date >> /home/roccflume/rebootlog.txt
pigs evt 3 &
sleep 60
ps -aux | grep -v grep | grep "pigs evt 3"
if [ $? -eq 0 ] ; then 
	echo 'pigs evt 3 forced reboot at' >> /home/roccflume/rebootlog.txt
	sudo reboot
	#pkill "pigs evt 3"
	#sudo pkill "rfm69app"
	#sudo /home/roccflume/.apps/./rfm69app 100 0 > /home/roccflume/rfm69runlog.txt"
fi
echo 'pigs evt 3 was successful' >> /home/roccflume/rebootlog.txt

