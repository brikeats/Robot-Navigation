
from scipy.interpolate import interp1d
import time
import numpy as np
import matplotlib.pyplot as plt



"""
Treatment here: http://planning.cs.uiuc.edu/node659.html

Units are cm, sec, radians.

## Robot parameters ##

# geometry
robot_width: distance between wheel planes (assumed parallel!)
wheel_radius: in cm

# sensor sweep
phi_range: the range of the sensor servo is -phi_range/2...phi_range/2
phi_n: number of steps to go from -phi_range/2...phi_range/2
phi_T: time to go from -phi_range/2...phi_range/2

## Robot state ##
pos: (x,y) coordinates in room frame
theta: angle of heading in coordinate frame (where heading is perpendicular to axis)
ur, ul: the speed of the left/right motors, assumed perfectly controllable


"""

### robot parameters
robot_width = 29.  # distance b/t wheels
robot_length = 17.5  # distance b/t caster and front axel
wheel_radius = 4.5
phi_range = np.pi/2
phi_n = 5
phi_T = 2.
sensor_range = 400.


### the room
room_width = 300.
room_height = 100.
walls = []
walls.append(np.array([0, room_width, 0, 0]))
walls.append(np.array([0, room_width, room_height, room_height]))
walls.append(np.array([0, 0, 0, room_height]))
walls.append(np.array([room_width, room_width, 0, room_height]))

# add other obstacles



def intersect(line1, line2):
    x1, x2, y1, y2 = line1
    x3, x4, y3, y4 = line2
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if den == 0:
        return None
    px = (x1*y2 - y1*x2) *(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)
    py = (x1*y2 - y1*x2) *(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)
    return np.array([px/den, py/den])



def logistic_curve(x, k=1):
    return 1./(1+np.exp(-k*x))


def reading_to_force(reading, stopping_distance=10, steepness=0.3):
    force = logistic_curve(reading-stopping_distance, steepness)-0.5
    return np.array(force)


def control_effort(readings):
    net_force = np.array([0., 0.])
    for phi, reading in readings.iteritems():
        force = reading_to_force(reading, stopping_distance=stopping_distance)
        net_force += force * np.array([np.cos(phi), np.sin(phi)])
    # x is foward, y is left

    theta = np.arctan2(net_force[1], net_force[0])
    x = np.pi*np.array([-1.0, -0.5, 0, 0.5, 1.0, 1.5, 2])
    yL = np.array([-1, 1, 1, -1, -1, 1, 1])
    yR = np.array([-1, -1, 1, 1, -1, -1, 1])
    uL = interp1d(x, yL)
    uR = interp1d(x, yR)

    return uL(theta), uR(theta)



def intersect_walls(pt1, pt2, walls):
    line = [pt1[0], pt2[0], pt1[1], pt2[1]]
    for wall in walls:
        intersect_pt = intersect(line, wall)
        if intersect_pt is not None:  # not parallel; intersection exists
            vec1 = pt2 - pt1
            vec2 = intersect_pt - pt1
            dot = np.dot(vec1, vec2)
            if dot > 0:  # pointing in right direction (forward)
                wall_vec = [wall[0]-wall[1], wall[2]-wall[3]]
                vec1 = intersect_pt - [wall[0], wall[2]]
                vec2 = intersect_pt - [wall[1], wall[3]]
                dot1 = np.dot(wall_vec, vec1)
                dot2 = np.dot(wall_vec, vec2)
                if dot1*dot2 <= 0:  # it is on the wall
                    vec = pt1 - pt2
                    vec1 = intersect_pt - pt1
                    vec2 = intersect_pt - pt2
                    dot1 = np.dot(vec, vec1)
                    dot2 = np.dot(vec, vec2)
                    if dot1*dot2 <= 0:  # it is between pt1 and pt2
    #                 TODO: return intersect_pt with smallest norm(vec2)
                        return intersect_pt


### Get range readings
def sense(pos, theta, walls, noise=0.):
    readings = {}
    for phi in np.linspace(theta-phi_range/2, theta+phi_range/2, phi_n):
        ds = sensor_range*np.array([np.cos(phi), np.sin(phi)])
        intersect_pt = intersect_walls(pos, pos+ds, walls)
        # TODO: restrict to phi_range
        vec = pos - intersect_pt
        dist = np.sqrt(vec[0]**2 + vec[1]**2)
        dist += np.random.rand()
        # the key is in the robot frame, since we assume robot has no map
        # so the range readings are associated with the servo angle
        readings[phi-theta] = dist + noise*np.random.randn()
    return readings


def control_effort(readings):
    net_force = np.array([0., 0.])
    for phi, reading in readings.iteritems():
        force = reading_to_force(reading, stopping_distance=30, steepness=0.05)
        net_force += force * np.array([np.cos(phi), np.sin(phi)])

    r = np.sqrt(net_force[0]**2 + net_force[1]**2)
    theta = np.arctan2(net_force[1], net_force[0])
    x = np.pi*np.array([-1.0, -0.5, 0, 0.5, 1.0, 1.5, 2])
    yL = np.array([-1, 1, 1, -1, -1, 1, 1])
    yR = np.array([-1, -1, 1, 1, -1, -1, 1])
    uL = interp1d(x, yL)
    uR = interp1d(x, yR)

    return r*uL(theta), r*uR(theta)


def draw_lines(lines, plots=None, colors=None, **kwargs):
    if colors is None:
        colors = ['k' for _ in range(len(lines))]
    if plots is None:
        line_plots = []
        for color, (ista, iend, jsta, jend) in zip(colors, lines):
            line_plot, = plt.plot([ista, iend], [jsta, jend], color=color, linewidth=3, **kwargs)
            line_plots.append(line_plot)
        return line_plots
    else:
        for plot, (ista, iend, jsta, jend) in zip(plots, lines):
            plot.set_data([ista, iend], [jsta, jend])


def robot_lines(pos, theta):
    # compute location of (left, right, back) corners of robot
    n = np.array([np.cos(theta-np.pi/2), np.sin(theta-np.pi/2)])  # heading
    L = pos + robot_width*n/2
    R = pos - robot_width*n/2
    h = np.array([np.cos(theta), np.sin(theta)])
    B = pos - robot_length*h
    front_line = np.array([L[0], R[0], L[1], R[1]])
    left_line = np.array([L[0], B[0], L[1], B[1]])
    right_line = np.array([R[0], B[0], R[1], B[1]])
    return [front_line, left_line, right_line]
    
    
def draw_robot(pos, theta, plots=None):
    lines = robot_lines(pos, theta)
    robot_colors = ['r','b','b']
    if plots is None:
        plot = draw_lines(lines, colors=robot_colors)
        return plot
    else:
        draw_lines(lines, plots, colors=robot_colors)


def check_for_collisions(pos, theta, walls):
    n = np.array([np.cos(theta-np.pi/2), np.sin(theta-np.pi/2)])  # heading
    L = pos + robot_width*n/2
    R = pos - robot_width*n/2
    h = np.array([np.cos(theta), np.sin(theta)])
    B = pos - robot_length*h

    if intersect_walls(L, R, walls) is not None:
        print 'front collision!'
        return True
    elif intersect_walls(L, B, walls) is not None:
        print 'left collision!'
        return True
    elif intersect_walls(R, B, walls) is not None:
        print 'right collision!'
        return True
    return False




### initial robot state
pos = np.array([room_width/2, 2*room_height/3])  # position (center of wheel axis)
theta = np.pi
ul, ur = -.5, 1.  # wheel speed in [rad/s]
stopping_distance = 30



# create initial plots
wall_plot = draw_lines(walls)
robot_plot = draw_robot(pos, theta)
readings = sense(pos, theta, walls)
effort = control_effort(readings)
plt.axis('equal')

dt = .1

for t in range(1, 200):

    robot_hit_wall = check_for_collisions(pos, theta, walls)

    if robot_hit_wall:
        break

    draw_robot(pos, theta, plots=robot_plot)
    readings = sense(pos, theta, walls, noise=1.)
    effort = control_effort(readings)
    ul, ur = effort


    ### velocity in robot frame
    fact = 10
    vx = fact * 0.5 * wheel_radius * (ul+ur) * np.cos(theta)
    vy = fact * 0.5 * wheel_radius * (ul+ur) * np.sin(theta)
    omega = wheel_radius / robot_width * (ur-ul)

    #     print effort
    #     plt.plot(pos[0], pos[1], 'o')

    ### propagate in time
    pos += [vx*dt, vy*dt]
    theta += omega*dt

    plt.draw()
    #     time.sleep(0.1)
