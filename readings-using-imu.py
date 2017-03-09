import serial
import math
import matplotlib.pyplot as plt
##initialisations
flag=0
angle=float(0.0)
x=float(0.0)
y=float(0.0)
z=float(0.0)
##connecting to the imu sensor
ard=serial.Serial('/dev/ttyUSB0',57600)
v=ard.readline()
a=raw_input("enter")#a pause
v=ard.readline()    #reading the sensor values
c=v.split(',')      #storing the values sent by imu in a list
init=float(c[4])    #the value which gives the yaw
print init
a=raw_input("enter")
test_init=init
test_init2=init
test_max=init
test_min=init
##to calibrate the sensor
while 1:
    v=ard.readline()
    c=v.split(',')
    test_new=float(c[4])
    print test_new
    if (test_new>test_init): #finding the maximum value given by the sensor
        test_max=test_new
        test_init=test_max
    if(test_new<test_init2): #finding the minimum value given by the sensor
        test_min=test_new
        test_init2=test_min
    if(math.fabs(test_new-init)<3 and math.fabs(test_max-test_min)>60 and test_min<-30 and test_max>30):
        break
print "test max is:",test_max,"test min is:",test_min
sens1=90/(test_min-init)
sens2=90/(test_max-init)
print sens1,sens2
v=ard.readline()
a=raw_input("enter")
for i in range(1,1500):
    print "wait"
init_1=0
while(math.fabs(init_1-init)>2):
    v=ard.readline()
    c=v.split(',')
    init_1=float(c[4])
    print "the value:",init_1-init
a=raw_input("enter")
offset=init_1
while 1:
    v=ard.readline()
    c=v.split(',')
    new=float(c[4])
    if(math.fabs(new-offset)<1):
        for i in range(1,1000):
              temp=9*9/3.1232313
        if(math.fabs(new-offset)<1):
            offset=(offset+new)/2
            print angle
            x+=math.sin(math.radians(angle))*1
            y+=math.cos(math.radians(angle))*1

            print "x-coordinate:",x,"y-coordinate",y
    else:
        if(new>init_1):
            angle=math.fabs(new-init)*sens2
            offset=float(c[4])
            flag=1
        elif(new<init_1):
            angle=math.fabs(new-init)*sens1
            offset=float(c[4])
