__author__ = 'Tennessee'

import pololu_maestro
import time
import math
import draw_solver
from pylab import *
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
import random
from math import atan2, floor

def get_intersection_2circ(p0, r0, p1, r1):
    p0x, p0y = tuple(p0)
    p1x, p1y = tuple(p1)

    d = sqrt(((p0x - p1x) ** 2.0 + (p0y - p1y) ** 2.0))
    if d > (r0 + r1):
        # Separate circles
        return []
    elif d < abs(r0 - r1):
        # Concentric circles
        return []
    elif d == 0 and r0 == r1:
        # Coincident circles
        return []
        
    a = ((r0 ** 2.0) - (r1 ** 2.0) + (d ** 2.0)) / (2.0 * d)
    
    h = sqrt((r0 ** 2.0) - (a ** 2.0))
    
    p2x = p0x + (a * (p1x - p0x)) / d
    p2y = p0y + (a * (p1y - p0y)) / d
    
    p3x_1 = p2x + (h * (p1y - p0y)) / d
    p3x_2 = p2x - (h * (p1y - p0y)) / d

    p3y_1 = p2y - (h * (p1x - p0x)) / d
    p3y_2 = p2y + (h * (p1x - p0x)) / d
    
    if p3x_1 == p3x_2 and p3y_1 == p3y_2:
        # Only one solution
        # TODO: Find better way to determine only one solution
        return [(p3x_1, p3y_1)]
    else:
        return [(p3x_1, p3y_1), (p3x_2, p3y_2)]
    
def find_highest_y(candidates):
    highest_y = candidates[0][1]
    idx = 0
    for cand_idx, p in enumerate(candidates):
        x, y = p
        if y > highest_y:
            highest_y = y
            idx = cand_idx
    
    return candidates[idx]
        
def drawbot_fk(pa, pb, ra, rb, la, lb, ext_co, ext_orth, theta1, theta2):
    # ext_co: extension coincident with la
    # ext_orth: extension orthogogonal to tip of ext_co    
    lp = sqrt((la + ext_co) ** 2.0 + ext_orth ** 2.0)
    alpha = atan2(ext_orth, (la + ext_co))
    pax, pay = pa
    pbx, pby = pb

    # Point at angle theta1 on periphery of circle A    
    cirax = pax + (ra * cos(theta1))
    ciray = pay + (ra * sin(theta1))
    
    # Point at angle theta2 on periphery of circle B
    cirbx = pbx + (rb * cos(theta2))
    cirby = pby + (rb * sin(theta2))
    
    # Find intersection of cirles of linkage la and lb
    c_candidates = get_intersection_2circ((cirax, ciray), la, (cirbx, cirby), lb)
    cx,cy = find_highest_y(c_candidates)
    
    # Find angles from linkages ra/rb tips to linkage la/lb tips
    phi1 = atan2((cy - ciray), (cx - cirax))
    phi2 = atan2((cy - cirby), (cx - cirbx))

    # Find offset pen position
    phi_p = phi1 + alpha
    
    # Find pen position
    px = cirax + (lp * cos(phi_p))
    py = ciray + (lp * sin(phi_p))    
    
    return (px, py, phi1, phi2, cx, cy)
    """
    for pair in pairs:
        px, py, phi1, phi2, cx, cy = drawbot_fk((-65.0, 0.0), (65.0, 0.0), 40.0, 40.0, 95.0, 95.0, 18.0, 9.5, pair[1], pair[0])
        x.append(px)
        y.append(py)
    """

def drawbot_ik(pa, pb, ra, rb, la, lb, ext_co, ext_orth, pe):
    # pe: Point of end effector (x, y) for which to find the theta1/theta2

    # ext_co: extension coincident with la
    # ext_orth: extension orthogogonal to tip of ext_co    
    lp = sqrt((la + ext_co) ** 2.0 + ext_orth ** 2.0)
    alpha = atan2(ext_orth, (la + ext_co))
    pax, pay = pa
    pbx, pby = pb
    pex, pey = pe

    # Find intersection between circle centered at `pe` of radius `lp`
    # and circle A. Keep highest y
    cira_candidates = get_intersection_2circ((pax, pay), ra, (pex, pey), lp)
    cirax,ciray = find_highest_y(cira_candidates)
    
    # Find linkage la/lb intersection point C from edge of circle a, 
    # on linkage la, offset alpha from the lp linkage angle
    lp_angle = atan2((pey - ciray), (pex - cirax))
    c_angle = lp_angle - alpha
    cx = cirax + (la * cos(c_angle))
    cy = ciray + (la * sin(c_angle))
    
    # Find two intersecting points from circle centered at la/lb intersection 
    # point C of radius lb, to circle B. Keep highest y
    cirb_candidates = get_intersection_2circ((cx, cy), lb, (pbx, pby), rb)
    cirbx,cirby = find_highest_y(cirb_candidates)
    
    # From points on cira/cirb on circle A/B, find linkage la/lb angles
    # theta1/theta2
    theta1 = atan2((ciray - pay), (cirax - pax))    
    theta2 = atan2((cirby - pby), (cirbx - pbx))
    
    return (theta1, theta2, cx, cy)


def angle_to_count(angle_rad, min_us, min_angle_rad, max_us, max_angle_rad):
    angle_rad = max(angle_rad, min_angle_rad)
    angle_rad = min(angle_rad, max_angle_rad)
    slope = (float(max_us) - float(min_us)) / (max_angle_rad - min_angle_rad)
    microsecs = min_us + ((angle_rad - min_angle_rad) * slope)
    counts = int(microsecs * 4.0)

    #print "%.2f,%d" % (microsecs, counts)
    return counts


def random_draw():
    random.seed(time.time())

    #maestro = pololu_maestro.PololuMaestro("COM6")
    #maestro.connect()

    pairs = []
    
    axis1_channel = 0
    axis2_channel = 1
    
    # Setup the drawing BY MAGIC RANDOM :)
    #omega1 = (math.pi / random.randint(50,200))
    #min1 = -math.pi / (2.0 + (random.random() * 2.0))
    #max1 = math.pi / (2.0 + (random.random() * 2.0))
    #angle1 = min1 + (random.random() * (max1 - min1))

    #omega2 = -math.pi / random.randint(50,250)
    #min2 = -math.pi / (2.0 + (random.random() * 2.0))
    #max2 = math.pi / (2.0 + (random.random() * 2.0))
    #angle2 = min2 + (random.random() * (max2 - min2))

    omega1 = (math.pi / random.randint(50,200))
    min1 = -math.pi / 4.0
    max1 = math.pi / 4.0
    angle1 = min1 + (random.random() * (max1 - min1))

    omega2 = -math.pi / random.randint(50,250)
    min2 = -math.pi / 4.00
    max2 = math.pi / 4.0
    angle2 = min2 + (random.random() * (max2 - min2))
    
    # Cool discontinuity
    # angle1=1.363
    # omega1=0.017
    # min1=-0.789
    # max1=1.421
    # angle2=0.489
    # omega2=-0.012
    # min2=-1.024
    # max2=0.909

    # Find Locus
    angle1 = min1
    omega1 = 0.1
    angle2 = min2
    omega2 = 0.001
    

    print "angle1=%.3f, omega1=%.3f, min1=%.3f, max1=%.3f" % (angle1, omega1, min1, max1)
    print "angle2=%.3f, omega2=%.3f, min2=%.3f, max2=%.3f" % (angle2, omega2, min2, max2)

    start_time = time.time()
    max_time = 60.0
    while (time.time() - start_time) < max_time:
        counts1 = angle_to_count(angle1, 600, -math.pi / 2, 2400, math.pi / 2)
        counts2 = angle_to_count(angle2, 600, -math.pi / 2, 2400, math.pi / 2)

        #maestro.set_target(axis1_channel, counts1)
        #maestro.set_target(axis2_channel, counts2)

        angle1 += omega1
        angle2 += omega2

        if angle1 > (max1):
            omega1 = -omega1
            angle1 += omega1

        elif angle1 < (min1):
            omega1 = -omega1
            angle1 += omega1

        if angle2 > (max2):
            omega2 = -omega2
            angle2 += omega2

        elif angle2 < (min2):
            omega2 = -omega2
            angle2 += omega2

        pairs.append((angle1, angle2))
        time.sleep(0.02)
    return pairs
#    maestro.close()
    
def split_line_into_segments(p1, p2, seg_length):
    segments = []    
    
    p1x, p1y = p1
    p2x, p2y = p2
    line_length = sqrt((p2x - p1x) ** 2.0 + (p2y - p1y) ** 2.0)
    
    intermediate_segments = int(floor(float(line_length) / float(seg_length)))
    if intermediate_segments == 0:
        intermediate_segments = 1
    
    for seg_idx in xrange(intermediate_segments):
        k = ((seg_idx * seg_length) / line_length)        
        x = p1x + k * (p2x - p1x)
        y = p1y + k * (p2y - p1y)
        segments.append((x,y))
        
    segments.append((p2x, p2y))
    
    return segments
    
def circle_draw():
    maestro = pololu_maestro.PololuMaestro("COM7")
    maestro.connect()

    OD = (-50.0, 0.0)
    OE = (50.0, 0.0)
    r1 = 40.0
    r2 = 40.0

    solver_theta = draw_solver.SolverTheta(math.pi/2.0, math.pi/2.0)

    for count in range(1):
        for phi1 in linspace(0, 2 * math.pi, 1):
            bx = 0.0 + (20.0 * math.cos(phi1))
            by = 90.0 + (20.0 * math.sin(phi1))
    
            theta1, theta2 = solver_theta.solve(bx, by)
            print theta1, theta2
            counts1 = angle_to_count(math.pi / 2.0, 500, 0, 2400, math.pi)
            counts2 = angle_to_count(math.pi / 2.0, 600, 0, 2400, math.pi)
    
            maestro.set_target(0, counts1)
            maestro.set_target(1, counts2)

            time.sleep(0.05)

    maestro.close()

        
if __name__ == "__main__":
    pa = (-65.0, 0.0)
    pb = (65.0, 0.0)
    ra = 40.0
    rb = 40.0
    la = 95.0
    lb = 95.0
    ext_co = 18.0
    ext_orth = 9.5
    
    p1 = (20.0, 80.0)
    p2 = (40.0, 100.0)
    seg_len = 1.0
    segment_points = split_line_into_segments(p1, p2, seg_len)
    
    fx = []
    fy = []
    ftheta1 = []
    ftheta2 = []
    ipairs = []
    
    for x, y in segment_points:
        # Find ik
        pe = (x, y)
        theta1, theta2, cx, cy = drawbot_ik(pa, pb, ra, rb, la, lb, ext_co, ext_orth, pe)
        ftheta1.append(theta1)
        ftheta2.append(theta2)

        # Get fk        
        pex, pey, phi1, phi2, cx, cy = drawbot_fk(pa, pb, ra, rb, la, lb, ext_co, ext_orth, theta1, theta2)
        
        fx.append(pex)
        fy.append(pey)    

    maestro = pololu_maestro.PololuMaestro("COM6")
    maestro.connect()

    for theta1, theta2 in [(3.0*pi/4.0, 3.0*pi/4.0), (pi/2.0, pi/2.0), (pi/4.0, pi/4.0)]:
        counts1 = angle_to_count(theta1, 500, 0, 2400, math.pi)
        counts2 = angle_to_count(theta2, 500, 0, 2400, math.pi)

        maestro.set_target(0, counts1)
        maestro.set_target(1, counts2)
        
        time.sleep(5)
   
   
    not_moved = True
    for x, y in segment_points:
        # Find ik
        pe = (x, y)
        theta1, theta2, cx, cy = drawbot_ik(pa, pb, ra, rb, la, lb, ext_co, ext_orth, pe)
    
        counts1 = angle_to_count(theta1, 500, 0, 2400, math.pi)
        counts2 = angle_to_count(theta2, 500, 0, 2400, math.pi)

        maestro.set_target(0, counts1)
        maestro.set_target(1, counts2)
   
        if not_moved:
            time.sleep(2.0)
            not_moved = False
        else:
            time.sleep(0.1)

    maestro.close()
    
    if False:
        f = figure()
        subplot(121)
        plot(fx, fy, 'r-')
        subplot(122)
        plot(ftheta1, ftheta2, 'b-o')
        #axes('square')
        show()
    
        # Compute feasible area
        feasx = []
        feasy = []
        px = []
        py = []    
        
        for theta1 in arange(pi, 0, -0.05):
            for theta2 in arange(0, pi, 0.05):
                try:
                    pex, pey, phi1, phi2, cx, cy = drawbot_fk(pa, pb, ra, rb, la, lb, ext_co, ext_orth, theta1, theta2)
                    feasx.append(cx)
                    feasy.append(cy)
                    px.append(pex)
                    py.append(pey)
                except IndexError:
                    # No solution founds
                    pass
                
        f = plt.figure()
        plt.plot(feasx, feasy, 'bo')
        plt.hold(True)    
        plt.plot(px, py, 'r.')
        c1 = plt.Circle((pa[0], pa[1]), ra, color='r')
        c2 = plt.Circle((pb[0], pb[1]), rb, color='r')
        plt.gcf().gca().add_artist(c1)
        plt.gcf().gca().add_artist(c2)    
        plt.xlim(-105, 105)
        plt.ylim(0, 120)    
        plt.show()
          
        #pairs = random_draw()
        #print split_line_into_segments((0,0), (1,1), 0.1)
        #print "===="
        #print split_line_into_segments((0,0), (1,0), 0.1)
        #print "===="    
        #print split_line_into_segments((0,0), (1,0), 0.1)
        #print "===="    
        #print split_line_into_segments((0,1), (0,0), 0.1)
        #print "===="
        #print split_line_into_segments((0,0), (1,1), 2)
        