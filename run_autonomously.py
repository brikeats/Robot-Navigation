
#!/usr/bin/python

from ServoController import ServoController
from ultrasonic_sensor import UltrasonicSensor
from motor_control import MotorController
from dual_mc33926_rpi import motors, MAX_SPEED

import threading
import RPi.GPIO as GPIO
import sys
import time
import numpy as np

servoNum = 0

servoHome = 1450
servoRange = np.pi/2  # [radians]
ticksPerRad = 573.
numAngles = 5

servoMin = int(round(servoHome - ticksPerRad*servoRange/2))
servoMax = int(round(servoHome + ticksPerRad*servoRange/2))


def targetToAngle(target):
	angle = servoRange * (target - servoHome) / (servoMax - servoMin)
	return angle
	# return np.rad2deg(angle)


# create a list of servo targets
targets1 = np.linspace(servoHome, servoMax, numAngles/2+1)
targets2 = np.linspace(servoMax, servoMin, numAngles)
targets3 = np.linspace(servoMin, servoHome, numAngles/2+1)
targets = np.hstack([targets1, targets2[1:], targets3[1:-1]])
targets = targets.astype(np.int)
angles = [targetToAngle(target) for target in np.unique(targets)]
readings = {angle:0 for angle in angles}


# get servo board ready
servoControl = ServoController()
servoControl.setSpeed(servoNum, 0)
servoControl.setAcceleration(servoNum, 15)

# get sensor ready
GPIO_pin = 15
sensor = UltrasonicSensor(GPIO_pin)

# get controller ready (readings -> motor effort)
controller = MotorController()
uL, uR = 0, 0

class MotorThread(threading.Thread):

	def __init__(self):
		super(MotorThread, self).__init__()
		self._stop = threading.Event()
		self.max_speed = int(round(0.75*MAX_SPEED))  # soft max, may be limited by battery output

	def run(self):
		print('starting motor thread')
		global uL
		global uR
		motors.enable()
		while not self.stopped():
			effortL = int(round(uL*self.max_speed)/1.75)
			effortR = int(round(uR*self.max_speed))
			motors.setSpeeds(effortR, -effortL)
			# motors.setSpeeds(2*self.max_speed, -self.max_speed)
			time.sleep(0.1)

	def stop(self):
		print('stopping motors')
		motors.setSpeeds(0, 0)
		motors.disable()
		self._stop.set()

	def stopped(self):
		return self._stop.isSet()
		

numSweeps = 15
motor_thread = MotorThread()
motor_thread.start()


sweep_num = 0
while True:
	try:
		for target in targets:
			angle = targetToAngle(target)

			# move servo to correct angle
			servoControl.setPosition(servoNum, target)
			while servoControl.getPosition(servoNum) != target:
				pass

			# measure range
			range_ = sensor.measurementInCM()
			readings[angle] = range_
			print('latest measurements:', readings.items())
			
			# update motor speeds
			if sweep_num == 0:
				uL, uR = 0, 0
			else:
				uL, uR = controller.control_effort(readings)
				print('percent efforts:', uL, uR)
		sweep_num += 1
		
	except KeyboardInterrupt:
		# go home at the end
		motor_thread.stop()
		servoControl.setPosition(servoNum, servoHome)
		time.sleep(0.2)
		sys.exit()

