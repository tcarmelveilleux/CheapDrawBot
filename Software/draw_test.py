__author__ = 'Tennessee'

import pololu_maestro
import time
import math
#import draw_solver
from pylab import *
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
import random
from math import atan2, floor
from chiplotle.tools.serialtools.virtual_serial_port import VirtualSerialPort
from chiplotle.plotters.plotter import Plotter
from chiplotle.geometry.core.coordinate import Coordinate
from chiplotle.geometry.core.coordinatearray import CoordinateArray
from chiplotle.tools.io import view, import_hpgl_file, save_hpgl, export
from chiplotle.hpgl.commands import PD, PU

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

def get_leon_points(ox, oy):
    all_a = "-929,-314,-897,-297,-865,-281,-832,-266,-800,-249,-801,-249,-796,-243,-791,-238,-790,-238,-769,-222,-759,-212,-762,-211,-763,-210,-759,-211,-737,-196,-736,-196,-726,-189,-726,-194,-731,-195,-732,-198,-733,-197,-734,-199,-729,-198,-728,-194,-726,-193,-727,-192,-710,-179,-678,-153,-669,-148,-670,-148,-662,-142,-663,-141,-660,-136,-658,-130,-657,-131,-646,-120,-651,-119,-651,-120,-652,-119,-647,-119,-642,-109,-641,-109,-635,-104,-636,-104,-620,-82,-599,-50,-598,-50,-582,-29,-583,-29,-578,-18,-577,-19,-571,-13,-566,-8,-567,-7,-564,1,-565,1,-562,9,-552,47,-551,46,-546,63,-546,62,-539,75,-540,75,-536,89,-533,113,-538,110,-538,108,-538,107,-536,105,-537,101,-537,100,-538,93,-534,96,-535,100,-536,101,-530,116,-531,116,-531,122,-531,138,-531,154,-530,153,-524,164,-526,164,-526,170,-526,186,-525,204,-526,204,-526,223,-527,223,-533,239,-531,239,-531,255,-531,271,-531,277,-533,276,-538,287,-536,287,-536,290,-537,290,-537,293,-541,289,-541,288,-543,288,-543,289,-546,289,-543,292,-546,300,-549,308,-551,305,-554,305,-557,305,-554,309,-553,309,-553,311,-553,314,-557,311,-557,310,-559,310,-562,310,-559,314,-558,314,-558,317,-558,319,-562,316,-562,315,-565,316,-565,315,-567,315,-566,318,-577,329,-578,326,-600,326,-626,326,-626,324,-636,319,-637,321,-640,321,-642,320,-639,317,-638,317,-639,314,-638,314,-638,311,-638,300,-641,301,-643,300,-644,302,-647,303,-647,304,-653,306,-653,305,-658,305,-656,301,-659,296,-661,290,-664,294,-666,294,-669,294,-666,290,-665,290,-665,287,-665,285,-668,286,-673,280,-684,270,-681,269,-681,263,-684,264,-695,254,-694,253,-699,243,-702,232,-709,210,-710,210,-715,199,-713,199,-713,194,-714,194,-720,178,-718,178,-718,170,-718,162,-720,162,-725,146,-724,146,-724,130,-726,130,-731,119,-729,119,-729,103,-729,87,-730,87,-736,71,-734,71,-734,55,-734,39,-734,-20,-734,-74,-734,-100,-733,-100,-728,-116,-729,-116,-729,-122,-729,-143,-730,-143,-730,-146,-729,-146,-728,-148,-725,-145,-725,-144,-722,-144,-720,-144,-723,-148,-723,-151,-724,-151,-724,-154,-722,-153,-717,-164,-719,-164,-719,-170,-718,-175,-716,-174,-705,-185,-708,-186,-708,-189,-708,-191,-704,-188,-704,-187,-701,-187,-698,-187,-702,-191,-702,-194,-703,-196,-700,-195,-689,-206,-692,-207,-692,-215,-692,-223,-688,-220,-688,-219,-685,-219,-682,-219,-685,-223,-686,-223,-686,-226,-686,-229,-682,-225,-680,-225,-677,-225,-680,-228,-681,-229,-681,-231,-681,-234,-677,-231,-677,-230,-674,-230,-672,-230,-675,-234,-676,-234,-676,-237,-676,-239,-672,-236,-672,-235,-669,-235,-666,-235,-669,-239,-670,-239,-670,-242,-670,-245,-666,-241,-664,-241,-661,-241,-664,-245,-665,-245,-665,-247,-665,-250,-662,-249,-657,-254,-660,-255,-660,-258,-659,-261,-656,-257,-653,-257,-650,-256,-651,-259,-646,-265,-636,-274,-636,-273,-624,-280,-624,-278,-618,-278,-619,-280,-608,-285,-608,-283,-602,-283,-602,-285,-586,-290,-586,-288,-581,-288,-581,-289,-575,-289,-576,-291,-568,-298,-569,-299,-566,-303,-565,-302,-561,-304,-561,-303,-549,-306,-549,-305,-533,-305,-517,-305,-517,-306,-501,-311,-501,-310,-485,-310,-453,-310,-436,-311,-436,-310,-420,-310,-421,-308,-410,-303,-410,-305,-392,-305,-372,-305,-373,-303,-362,-298,-330,-287,-319,-282,-298,-275,-299,-275,-290,-270,-291,-270,-277,-260,-280,-258,-280,-255,-280,-253,-277,-254,-267,-246,-266,-246,-255,-238,-256,-238,-238,-220,-239,-220,-224,-200,-227,-199,-227,-194,-224,-195,-213,-184,-216,-183,-216,-173,-216,-162,-214,-162,-209,-151,-211,-151,-211,-130,-210,-130,-210,-127,-211,-127,-211,-124,-215,-128,-217,-128,-220,-128,-217,-124,-216,-124,-216,-122,-216,-119,-219,-120,-240,-99,-241,-102,-247,-102,-246,-99,-251,-93,-252,-96,-279,-96,-300,-96,-300,-98,-308,-103,-304,-105,-301,-102,-299,-103,-299,-102,-295,-103,-299,-106,-299,-108,-301,-107,-304,-110,-305,-107,-308,-107,-311,-107,-310,-110,-315,-115,-336,-137,-338,-133,-340,-133,-340,-134,-343,-134,-340,-138,-339,-138,-339,-140,-339,-143,-341,-143,-346,-151,-348,-147,-345,-144,-345,-142,-345,-138,-349,-142,-350,-142,-350,-144,-353,-147,-350,-148,-350,-159,-351,-159,-362,-191,-360,-191,-360,-196,-362,-196,-367,-207,-366,-207,-366,-234,-364,-234,-359,-250,-360,-250,-360,-261,-359,-260,-353,-271,-355,-271,-355,-277,-352,-276,-347,-281,-325,-302,-328,-303,-329,-306,-328,-306,-328,-309,-324,-305,-322,-305,-319,-304,-319,-307,-311,-311,-315,-313,-318,-311,-320,-311,-324,-311,-320,-315,-320,-316,-318,-315,-315,-319,-314,-317,-311,-317,-311,-315,-308,-315,-305,-314,-305,-316,-303,-317,-307,-319,-307,-322,-307,-325,-303,-322,-303,-321,-298,-322,-298,-321,-292,-320,-293,-323,-282,-328,-281,-326,-265,-326,-233,-326,-228,-326,-228,-324,-218,-319,-217,-321,-212,-321,-212,-320,-196,-314,-180,-309,-180,-308,-170,-303,-169,-305,-164,-305,-164,-303,-154,-298,-153,-300,-150,-300,-150,-299,-148,-299,-151,-295,-152,-295,-152,-293,-152,-290,-148,-292,-143,-289,-143,-290,-137,-287,-138,-286,-100,-260,-102,-259,-96,-251,-91,-243,-90,-244,-82,-238,-74,-233,-63,-225,-53,-217,-56,-215,-56,-213,-56,-210,-52,-213,-49,-213,-49,-214,-46,-215,-47,-211,-37,-200,-40,-199,-40,-196,-39,-194,-36,-197,-36,-198,-33,-197,-33,-198,-30,-198,-31,-195,-26,-190,-15,-179,-5,-168,-4,-171,-1,-171,2,-171,-1,-167,-2,-167,-2,-164,-2,-162,2,-165,2,-166,5,-166,7,-166,4,-162,3,-162,3,-159,3,-156,7,-158,22,-148,23,-149,25,-149,23,-153,20,-154,20,-155,19,-155,18,-159,22,-158,23,-158,24,-155,28,-154,28,-153,39,-148,39,-150,77,-150,119,-150,125,-150,125,-149,128,-149,128,-150,130,-150,126,-154,126,-156,126,-159,130,-156,130,-155,133,-155,135,-155,132,-159,131,-159,131,-162,131,-164,135,-161,135,-160,138,-160,141,-160,138,-164,137,-164,137,-167,137,-170,139,-169,144,-176,146,-172,142,-169,142,-164,141,-162,145,-165,144,-170,143,-170,149,-186,148,-186,148,-191,150,-190,161,-201,158,-202,158,-234,158,-250,155,-249,147,-257,139,-265,142,-266,143,-269,142,-269,142,-271,138,-268,138,-267,135,-267,133,-267,136,-271,137,-271,137,-274,137,-277,133,-274,133,-273,130,-273,127,-273,131,-277,131,-279,131,-282,128,-279,127,-278,125,-278,122,-278,125,-282,126,-282,126,-285,126,-287,122,-284,120,-285,120,-283,117,-283,111,-283,69,-283,5,-283,-1,-283,-1,-284,-4,-284,-4,-283,-6,-283,-2,-279,-2,-277,-2,-274,-6,-277,-6,-278,-9,-278,-11,-278,-11,-276,-20,-271,-15,-269,-12,-272,-11,-271,-10,-272,-7,-272,-10,-268,-10,-267,-13,-267,-15,-264,-17,-267,-20,-267,-22,-267,-19,-263,-18,-263,-18,-253,-19,-253,-25,-237,-24,-237,-24,-226,-24,-199,-24,-178,-20,-181,-20,-182,-17,-181,-17,-182,-14,-182,-15,-179,-10,-174,-9,-175,-4,-172,1,-169,-1,-167,-1,-165,-2,-165,-3,-162,1,-163,6,-158,7,-159,17,-153,18,-155,23,-155,22,-152,33,-142,34,-144,39,-144,39,-143,50,-137,50,-139,56,-140,56,-139,61,-139,57,-135,57,-132,57,-130,61,-133,61,-134,77,-134,76,-132,93,-127,93,-128,98,-128,98,-127,108,-121,109,-123,114,-123,114,-122,130,-116,130,-118,143,-117,157,-118,189,-118,200,-118,205,-118,205,-119,221,-124,221,-123,226,-123,238,-123,237,-124,243,-126,242,-127,247,-131,244,-132,244,-135,244,-138,248,-135,251,-135,251,-134,256,-133,260,-132,260,-134,264,-135,260,-138,260,-140,260,-143,264,-140,266,-141,266,-139,269,-138,272,-137,272,-139,274,-140,270,-143,270,-146,270,-148,274,-145,274,-144,277,-144,280,-144,277,-148,276,-148,276,-159,276,-170,278,-169,283,-178,284,-173,282,-171,282,-169,282,-165,286,-168,287,-168,286,-171,290,-174,287,-175,287,-180,286,-186,288,-185,294,-196,292,-196,292,-213,292,-255,292,-271,293,-271,293,-275,291,-274,291,-276,288,-272,291,-269,290,-267,292,-267,293,-263,292,-263,292,-258,292,-215,292,-205,294,-205,299,-194,297,-194,297,-183,299,-184,304,-173,303,-172,303,-167,304,-167,309,-151,307,-151,307,-148,308,-148,308,-146,312,-149,312,-150,315,-150,317,-150,314,-146,313,-146,313,-143,313,-140,317,-144,320,-144,323,-144,319,-140,319,-138,319,-135,322,-137,327,-134,327,-135,333,-132,332,-131,340,-123,348,-115,349,-118,355,-118,354,-116,365,-111,365,-112,392,-112,419,-112,418,-115,423,-120,424,-117,427,-117,427,-118,429,-118,426,-122,427,-124,425,-124,425,-127,425,-159,428,-158,434,-163,439,-169,436,-170,436,-184,436,-196,440,-193,440,-192,443,-192,446,-192,442,-196,442,-207,442,-218,444,-217,450,-222,447,-223,447,-250,446,-250,446,-253,447,-253,448,-255,451,-251,454,-251,456,-251,453,-255,452,-255,452,-258,452,-261,456,-258,456,-257,459,-257,462,-257,458,-261,458,-263,458,-266,461,-263,462,-262,464,-262,467,-262,464,-266,463,-266,463,-269,463,-271,467,-268,469,-269,469,-267,472,-266,475,-266,475,-268,477,-268,474,-271,474,-274,474,-277,477,-274,478,-273,480,-273,483,-273,480,-277,479,-277,479,-279,479,-282,483,-279,483,-278,485,-278,488,-278,487,-281,492,-286,494,-283,496,-283,499,-284,496,-287,495,-287,495,-290,495,-293,499,-290,499,-289,501,-290,501,-289,504,-288,504,-289,531,-289,531,-290,547,-295,547,-294,563,-294,601,-294,600,-293,616,-287,628,-281,629,-281,643,-277,643,-278,651,-279,651,-278,659,-278,659,-276,675,-265,675,-264,691,-248,722,-216,723,-217,734,-212,733,-211,743,-200,744,-200,754,-190,755,-191,766,-186,767,-186,777,-183,776,-182,785,-178,784,-177,792,-168,790,-167,793,-159,796,-151,797,-152,807,-141,819,-131,818,-130,823,-122,829,-114,830,-115,840,-107,851,-99,861,-88,862,-89,873,-84,872,-83,893,-61,904,-51,903,-50,914,-34,915,-35,936,-13,933,-12,933,-7,933,-1"
    pairs = []
    count = 0
    x = 0
    y = 0

    px = []
    py = []
    
    for point in all_a.split(","):
        p = float(point) / 1016.0 * 25.4 * 1.5
        if count % 2 == 0:
            x = ox + p
            px.append(x)
        else:
            y = oy + p
            py.append(y)
            pairs.append((x,y))
        count += 1
        
    print pairs
    f = plt.figure()
    plt.plot(px,py,'r-')
    plt.show()
    
    return pairs

def gen_leon_path():
    leon_points = get_leon_points(5.0, 100.0)
    return gen_path(leon_points)

def draw_path(thetas, maestro_com_port, delay=0.005):
    maestro = pololu_maestro.PololuMaestro(maestro_com_port)
    maestro.connect()
    try:
        pen_up(maestro)
        
        not_moved = True
        for theta1, theta2 in thetas:
            counts1 = angle_to_count(theta1, MIN_US_A, MIN_ANGLE_RAD_A, MAX_US_A, MAX_ANGLE_RAD_A)
            counts2 = angle_to_count(theta2, MIN_US_B, MIN_ANGLE_RAD_B, MAX_US_B, MAX_ANGLE_RAD_B)
    
            maestro.set_target(0, counts1)
            maestro.set_target(1, counts2)
       
            if not_moved:
                time.sleep(1.0)
                pen_down(maestro)
                not_moved = False
            else:
                time.sleep(delay)
                
    finally:
        maestro.close()

def draw_hpgl(filename, maestro_com_port, delay=0.02, seg_len=0.1):
    cheap_draw_bot = CheapDrawBotKinematics()

    hpgl_data = import_hpgl_file(filename)

    prev_x = 0
    prev_y = 0
    pen_is_up = False

    maestro = pololu_maestro.PololuMaestro(maestro_com_port)
    maestro.connect()
    try:
        accumulated_path = []
        for command in hpgl_data:
            if isinstance(command, PU):
                pex, pey = cheap_draw_bot.local_coord_from_hpgl((command.x[0], command.y[0]))
                accumulated_path = [(pex, pey)]
                pen_up(maestro)
                pen_is_up = True
            elif isinstance(command, PD):
                # Pen down here
                for coord in command.xy:
                    pex, pey = cheap_draw_bot.local_coord_from_hpgl((coord.x, coord.y))
                    accumulated_path.append((pex, pey))

                points, thetas = cheap_draw_bot.gen_path(accumulated_path, seg_len)

                for theta1, theta2 in thetas:
                    counts1 = angle_to_count(theta1, MIN_US_A, MIN_ANGLE_RAD_A, MAX_US_A, MAX_ANGLE_RAD_A)
                    counts2 = angle_to_count(theta2, MIN_US_B, MIN_ANGLE_RAD_B, MAX_US_B, MAX_ANGLE_RAD_B)

                    maestro.set_target(0, counts1)
                    maestro.set_target(1, counts2)

                    if pen_is_up:
                        time.sleep(1.0)
                        pen_down(maestro)
                        pen_is_up = False
                    else:
                        time.sleep(delay)
    finally:
        maestro.close()

def test_hpgl():
    draw_hpgl("Funstuff.hpgl", "COM27")

def test_leon_path():
    cheap_draw_bot = CheapDrawBotKinematics()
    for i in range(4):
        leon_points = get_leon_points(5.0, 100.0)
        points, thetas = cheap_draw_bot.gen_path(leon_points)
        draw_path(thetas, "COM6")

def test_feasible_area():
    cheap_draw_bot = CheapDrawBotKinematics()
    feas_points, feas_thetas = cheap_draw_bot.get_work_area(save_hpgl="feasible.plt", plot=True)
    # draw_path(feas_thetas, "COM6", delay=0.1)

def draw_spiro():
    # Center of circle: (pax + 79, pay + 92.1, radius 31
    #
    # TODO: https://en.wikipedia.org/wiki/Spirograph
    pass


def plotify(pathlist):
    all_x = []
    all_y = []
    for x, y in pathlist:
        all_x.append(x)
        all_y.append(y)

    return all_x, all_y


def test_concave_hull():
    cheap_draw_bot = CheapDrawBotKinematics()
    xy_resolution = 0.5
    feas_points, feas_thetas = cheap_draw_bot.get_work_area(xy_resolution=xy_resolution)

    SAMPLE_POINTS = [
        (0, 0), (1, 0), (2, 0), (3, 0),
        (0, 1), (1, 1), (2, 1), (3, 1),
        (0, 2), (1, 2), (2, 2), (3, 2),
        (0, 3), (3, 3), # (1, 3), (2, 3),
    ]

    all_xy = feas_points
    wheel_radius = xy_resolution * 5.0
    outline_points, circle_centers = concave_hull_wheel(all_xy, wheel_radius)

    f = plt.figure()

    ax = f.add_subplot(111)
    all_x, all_y = plotify(all_xy)
    out_x, out_y = plotify(outline_points)

    plt.plot(all_x, all_y, "ro")
    plt.plot(out_x, out_y, "b-")

    for idx, cir in enumerate(circle_centers):
        c = plt.Circle((cir[0], cir[1]), wheel_radius, fill=False, color='g')
        t = plt.Text(x=cir[0], y=cir[1], text="%d" % idx)
        ax.add_artist(c)
        ax.add_artist(t)
    plt.axis('equal')

    plt.show()

def main():
    #test_hpgl()
    #test_leon_path()
    #test_feasible_area()
    test_concave_hull()

    if False:
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
                
if __name__ == "__main__":
    main()    
