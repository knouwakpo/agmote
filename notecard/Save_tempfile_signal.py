import pigpio

mypigpio = pigpio.pi()

#Triggers an event to signal to the background data
# processing program to save the radiodata.txt file to 
# radiodata.temp.x.txt file (x ranges from 0 to 255).
#These files will be read, and the data sent to notehub
mypigpio.event_trigger(3)
