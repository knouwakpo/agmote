#!/usr/bin/bash

#Regularly check changes on the "radiodata.txt" file to ensure that it updates with new data
#Reboots if no update has occured in 30min. This indicates that the host radio may have stalled.

rebootflag="/home/roccflume/.apps/REBOOTED"
if [ -f $rebootflag ]; then
	echo 'Rebooted at '$(date) >> /home/roccflume/rebootlog.txt
	rm $rebootflag
else

	moddate_=$(stat /home/roccflume/.apps/radiodata.txt | grep Modify)
	mydate=$(($(date -d "$(echo $(echo $moddate_ | sed 's/\(\.[0-9]\+\)\|\(Modify:\)\|\(\-[0-9]\{4\}\)//g'))" +%s) + 2000))
	nowdate=$(date +%s)

	if [[ $mydate -lt $nowdate ]]; 

	then 
		echo 'Inactivity on radiodata.txt forced reboot at '$(date) >> /home/roccflume/rebootlog.txt
		pkill "pigs evt 3"
		sudo pkill "rfm69app"
		echo "Rebooting at "$(date) >> $rebootflag
		sudo reboot

	fi
fi

