#!/usr/bin/env python
#Readings to get from Inertial Measurement unit using Robot Operatng system Framework
import time
import serial
import rospy
import math
from geometry_msgs.msg import Vector3 
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
current_milli_time = lambda: (time.time() * 1000)
frequency = 80
degrees2rad=(math.pi/180.0)
isVerbose=False
rospy.init_node('imu_pub', anonymous=True)


ser = serial.Serial("/dev/ttyUSB0",57600)

	
def talker():
	yaw=0
	pitch=0
	roll=0
	x=0
	y=0
	z=0
	delta=0
	timeX=0
	timeY=0
	prevTimeX=0
	prevTimeY=0
	maxi=-999
	mini=999
	avg=0
	avg2=0
	avg3=0
	vx=0
	vy=0
	vz=0
	flag=False
	count=1
	
	samples=[]
	samples2=[]
	samples3=[]
	pub=rospy.Publisher('imu', Imu, queue_size=1)
	imu=Imu()
	print 'Testing serial communication'
	print ser.inWaiting()
	print 'Flushing from buffer: ',ser.read(ser.inWaiting())
	timeStart=current_milli_time()
	rate = rospy.Rate(frequency)
	while not rospy.is_shutdown():
		while ser.inWaiting() > 0:
						
			line=ser.readline()
			print line
			if isVerbose:
				print 'No of bytes : ',ser.inWaiting()	
				print line
				print 'ASCII array:',
				for c in line:
					print ord(c),	
			else:
				print line
			line = line.replace("#YPR=","")
			line = line.replace("I","")
			line = line.replace("U","")
			line = line.replace("/r","")
			line = line.replace("/n","")
			print line
			parts=line.split(',')
			
                  	try:   
				imu.linear_acceleration.x=-float(parts[3])*(9.806 / 256.0)#-0.87#avg
				imu.linear_acceleration.y= float(parts[4])*(9.806 / 256.0)#-avg2
				imu.linear_acceleration.z= float(parts[5])*(9.806 / 256.0)#-avg3  			        
				yaw_deg = -float(parts[0])
    			        yaw_deg = yaw_deg# + imu_yaw_calibration
        		        if yaw_deg > 180.0:
            			  	yaw_deg = yaw_deg - 360.0
      			        if yaw_deg < -180.0:
      		       	        	yaw_deg = yaw_deg + 360.0
   			        yaw = yaw_deg*degrees2rad
   
       				pitch = -float(parts[1])*degrees2rad
  			        roll  =  float(parts[2])*degrees2rad	
#				if float(parts[3]>12 or parts[3]<-12): 	        	
#					imu.linear_acceleration.x=-0#(float(parts[3]))* (9.806 / 256.0)
#				else:

				prevTimeX=timeX
				timeX=current_milli_time()
				
				delta=(timeX-prevTimeX)/1000
#				imu.angular_velocity.x=(float(parts[6]))*(98.0/32767.0)*(math.pi/180)*3
#				imu.angular_velocity.y=(float(parts[7]))*(98.0/32767.0)*(math.pi/180)*3
#				imu.angular_velocity.z=-(float(parts[8]))*(98.0/32767.0)*(math.pi/180)*3
#			
			except ValueError or IndexError or IOError:
				print 'Invalid readings'
				continue
			samples.append(-float(parts[3])*(9.806/256.0))
			samples2.append((float(parts[4]))*(9.806 / 256.0))
			samples3.append(float(parts[5])*(9.806/256.0))
			if len(samples)>3000:
				del samples[0:1]
			s0 = sum(1 for x in samples)
			s1 = sum(x for x in samples2)
			s2 = sum(x*x for x in samples3)

			x+=vx*(delta)#+0.5*imu.linear_acceleration.x*delta*delta
			y+=vy*(delta)#+0.5*imu.linear_acceleration.y*delta*delta
			z+=vz*(delta)#+0.5*imu.linear_acceleration.z*delta*delta

			vx+=math.sqrt(imu.linear_acceleration.x**2+imu.linear_acceleration.y**2)*delta*math.cos(yaw)
			vx+=math.sqrt(imu.linear_acceleration.x**2+imu.linear_acceleration.y**2)*delta*math.sin(yaw)
			vz+=imu.linear_acceleration.z*(delta)
			
#			x+=vx*delta
#			y+=vy*delta
			if not flag:
				vx=0
				vy=0
				vz=0
				x=0
				y=0
				z=0
				count=count+1
			
#			try:
#				std_dev = math.sqrt((s0 * s2 - s1 * s1)/(s0 * (s0 - 1)))
#			except ZeroDivisionError or ValueError:
			std_dev=-1	
			q = quaternion_from_euler(roll,pitch,yaw)
	       	        imu.orientation.x = q[0]
		        imu.orientation.y = q[1]
		        imu.orientation.z = q[2]
		        imu.orientation.w = q[3]	

#			if abs(imu.linear_acceleration.x)<0.06:
#				imu.linear_acceleration.x=0
#			if abs(imu.linear_acceleration.y)<0.06:
#				imu.linear_acceleration.y=0

			imu.header.frame_id='base_link'
			imu.header.stamp=rospy.Time.now()
			
			vx+=math.sqrt(imu.linear_acceleration.x**2+imu.linear_acceleration.y**2)*delta*math.cos(yaw)
			vx+=math.sqrt(imu.linear_acceleration.x**2+imu.linear_acceleration.y**2)*delta*math.cos(yaw)
						
			print "\n"
			print "ax :",imu.linear_acceleration.x
			print "ay :",imu.linear_acceleration.y
			print "Delta :",delta 		
			print "vx : ",vx," vy : ",vy," vz : ",vz
			if imu.linear_acceleration.x>maxi:
				maxi=imu.linear_acceleration.x
			if imu.linear_acceleration.x<mini:
				mini=imu.linear_acceleration.x
			print timeStart," ",timeX," ",timeX-timeStart			
			if timeX-timeStart<10000:
				avg=sum(samples)/len(samples)	
				avg2=sum(samples2)/len(samples2)
				avg3=sum(samples3)/len(samples3)
			else :
				flag=True
			print "Minimum : ",mini,"\nStandard deviaion : ",std_dev,"\nMaximum : ",maxi,"\nAverage",avg," ",avg2," ",avg3
		
			print "(x,y,z) : " ,(x,y,z)
			pub.publish(imu)
			print "Published : ",imu
			#rate.sleep()
			

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

