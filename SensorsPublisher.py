#!/usr/bin/env python
# license removed for brevity
import serial
import time
import rospy
from std_msgs.msg import String

PORT = '/dev/ttyUSB3'
SPEED = 9600
connection = serial.Serial(PORT, SPEED, timeout=0.050)

def talker():
	pub_top_value = rospy.Publisher('/Sensor_value', String, queue_size=10)
	rospy.init_node('Sensor_value', anonymous=True)
	rate = rospy.Rate(20) # 20hz
	while not rospy.is_shutdown():
		Force = connection.readline()
		pub_top_value.publish(Force.decode('utf-8'))
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
