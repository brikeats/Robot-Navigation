#!/usr/bin/env python

import pickle as pkl
import numpy as np
import RPi.GPIO as GPIO
import time
import sys


class UltrasonicSensor:

    def __init__(self, pi_signal_pin):
        GPIO.setmode(GPIO.BOARD)
        self.pi_signal_pin = pi_signal_pin


    def scan(self, num_scans=1):

        measurements = []
        min_target = 1000
        max_target = 2000

        for _ in range(num_scans):

            print '---------'
            print 'Going to', min_target,
            sys.stdout.flush()
            self.servo_controller.setPosition(self.servo_pin, min_target)
            while self.servo_controller.getPosition(self.servo_pin) != min_target:

                pwm = self.servo_controller.getPosition(self.servo_pin)
                print 'pwm before', pwm

                # block until the pattern is visible
                print 'Pinging...',
                sys.stdout.flush()
                distance = self.measurementInCM()
                print 'got', distance
                print 'pwm after', self.servo_controller.getPosition(self.servo_pin)
                sys.stdout.flush()

                print 'adding [%i, %f] to measurements' % (pwm, distance)
                measurements.append([pwm, distance])



            print '---------'
            print 'Going to', max_target,
            sys.stdout.flush()

            self.servo_controller.setPosition(self.servo_pin, max_target)
            while self.servo_controller.getPosition(self.servo_pin) != max_target:

                pwm = self.servo_controller.getPosition(self.servo_pin)
                print 'pwm before', pwm

                # block until the pattern is visible
                print 'Pinging...',
                sys.stdout.flush()
                distance = self.measurementInCM()
                print 'got', distance
                print 'pwm after', self.servo_controller.getPosition(self.servo_pin)
                sys.stdout.flush()

                print 'adding [%i, %f] to measurements' % (pwm, distance)
                measurements.append([pwm, distance])


        self.servo_controller.setPosition(self.servo_pin, (min_target+max_target)/2)
        print 'returning %i measurements' % len(measurements)
        GPIO.cleanup()
        return measurements


    def measurementInCM(self):

        # send ping
        GPIO.setup(self.pi_signal_pin, GPIO.OUT)
        GPIO.output(self.pi_signal_pin, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(self.pi_signal_pin, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(self.pi_signal_pin, GPIO.LOW)

        # setup self.pi_signal_pin as input
        GPIO.setup(self.pi_signal_pin, GPIO.IN)

        # get duration from Ultrasonic SIG pin
        while GPIO.input(self.pi_signal_pin) == 0:
            start = time.time()
        while GPIO.input(self.pi_signal_pin) == 1:
            stop = time.time()


        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound (cm/s)
        speed_of_sound = 34300
        distance = (stop-start) * speed_of_sound / 2  # distance there and back (divide by 2)

        return distance




if __name__ == '__main__':

    GPIO_pin = 15

    sensor = UltrasonicSensor(GPIO_pin)
    measurements = sensor.scan(5)

    print 'writing %i measurements' % len(measurements)
    with open('ultrasonic_scan.pkl', 'w') as f:
        pkl.dump(measurements, f)