import numpy as np


def sigmoid(x, k=1):
    return 1./(1+np.exp(-k*x))


"""
This class controls the wheel effort. Its main function is to map sensor
readings to efforts for the wheel motors. 
"""
class MotorController:
    
    def __init__(self):
        self.start_turning_thresh = 0.05
        self.stop_turning_thresh = 4*self.start_turning_thresh
        self.force_ewma = self.stop_turning_thresh  # don't start at 0 or it will execute turnabout immediately
        self.alpha = 0.8
        self.turn_about_mode = False
    
    def readings_to_force(self, readings, stopping_distance=25, steepness=0.1):
        net_force = np.array([0., 0.])
        for phi, reading in readings.items():
            force = 2*(sigmoid(reading-stopping_distance, steepness)-0.5)
            net_force += force * np.array([np.cos(phi), np.sin(phi)])
        max_fx = np.sum(np.cos(phi) for phi in readings.keys())
        max_fy = np.sum(np.abs(np.sin(phi)) for phi in readings.keys())
        net_force[0] /= max_fx
        net_force[1] /= max_fy
        return net_force
    
    def turn_about_is_finished(self, readings):
        force = self.readings_to_force(readings)
        f = force[0]**2 + force[1]**2
        if f < self.stop_turning_thresh:
            return False
        else:
            return True

    def control_effort(self, readings, scale=1):
        
        if self.turn_about_mode:
            net_force = np.array([0, -0.5])
            if self.turn_about_is_finished(readings):
                self.turn_about_mode = False
        else:
            net_force = self.readings_to_force(readings)
        print('net_force', net_force)

        
        # check if it's stuck (consistently small net_force)
        self.force_ewma = (1-self.alpha)*net_force + self.alpha*self.force_ewma
        f_ewma = self.force_ewma[0]**2 + self.force_ewma[1]**2
        f = net_force[0]**2 + net_force[1]**2
        if f_ewma < self.start_turning_thresh:
            self.turn_about_mode = True

        # handle the case with a corner near the edge of the FOV
        if np.abs(net_force[1]) > 0.3:
            net_force[0] = -0.3

        net_force *= float(scale)     
        v = net_force[0]
        omega = net_force[1]
        uR = v - omega
        uL = v + omega
        return uL, uR
    

# range_readings_deg = {0.0: 33.28814999199494, 22.5: 57.74405000433944, -45.0: 25.416300009919723, 45.0: 34.23139999936211, -22.5: 25.604949995795323}
# # range_readings = {np.deg2rad(key): val for key, val in range_readings_deg.items()}
# range_readings = {np.deg2rad(key): 100000 for key in range_readings_deg.keys()}

# controller = MotorController()
# uL, uR = controller.control_effort(range_readings)
# print ('uL, uR = ', uL, uR)
