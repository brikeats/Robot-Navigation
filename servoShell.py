from ServoController import ServoController


servoControl = ServoController()


# servo[i] = j means logical servo i is plugged into pin j
numServos = 4
channels = [0 for i in range(numServos)]
channels[0] = 0
channels[1] = 1
channels[2] = 2
channels[3] = 3

# home (logical indices)
home = [0 for i in range(numServos)]
home[0]  = 1519

home[2]  = 1525
home[3]  = 1518

# servo0 range is 165, (max,min) = (1360,1690)
servoRange = 150


while True:
    servoNum = input('Enter servo number (0-'+str(numServos-1)+') >>')
    try:
        servoNum = int(servoNum)
        if servoNum < 0 or servoNum > numServos:
            print('Invalid servo number')
            continue
        print('logical servoNum = '+str(servoNum))
        channel = channels[servoNum]
        print('(channel = '+str(channel)+')')  
        break
    except:
        print('Invalid servo number')
        continue
    
    
servoControl.setSpeed(servoNum, 0)
servoControl.setAcceleration(servoNum, 8)

print('Ctl-C to quit')


def level_to_target(level, servoNum):
    """
    Maps percentage -100...100 to 
    """
    if level > 100:
        print 'Setting level to 100'
        level = 100
    if level < -100:
        print 'Setting level to -100'
    target = servoRange*level/100 + home[servoNum]
    return target
    

while True:
    target = input("Enter target >>")
    # level = input("Enter level (-100...100) >>")
    try:
        # level = int(level)
        # target = level_to_target(level, channel)
        # servoControl.setPosition(channel, target)
        target = int(target)
        servoControl.setPosition(channel, target)
        print('Target set to '+str(target))
    except ValueError:
        print('Invalid target')
    
    
    
    
    