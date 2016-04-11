
import time
from dual_mc33926_rpi import motors, MAX_SPEED


try:
    motors.enable()
    motors.setSpeeds(0, 0)

    active_motor = 1
    max_speed = MAX_SPEED/2  # soft max, may be limited by battery output

    print 'Motor control shell. Enter a number -100...100 to set motor speed.',
    print '"m1" or "m2" to select which motor to control, "q" to quit.'

    while True:
        prompt = 'motor %i >> ' % active_motor
        cmd = raw_input(prompt)

        if cmd in ['q', 'Q']:
            break
        if cmd in ['M1', 'm1']:
            print 'motor 1 active'
            active_motor = 1
            continue
        if cmd in ['M2', 'm2']:
            print 'motor 2 active'
            active_motor = 2
            continue
        try:

            # get desired speed
            speed = int(cmd)
            if speed > 100:
                print 'speed is greater than 100'
                speed = 100
            if speed < -100:
                print 'speed is less than -100'
                speed = -100

            # set the speed
            if active_motor == 1:
                print 'setting motor 1 speed to ', speed
                print type(max_speed*speed/100)
                motors.motor1.setSpeed(max_speed*speed/100)
            else:
                print 'setting motor 2 speed to ', speed
                motors.motor2.setSpeed(max_speed*speed/100)

        except ValueError:
            print 'unrecognized command'


finally:
    # Stop the motors, even if there is an exception
    # or the user presses Ctrl+C to kill the process.
    print
    print 'Cleaning up'
    motors.setSpeeds(0, 0)
    motors.disable()
