#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
CheapDrawBot robot controller

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""

import pololu_maestro
import time
import json
from numpy import pi, arange
import numpy as np
from utils.geometry import *
from utils.hpglutils import save_path_as_hpgl
from utils.plotutils import plot_feasible_xy_theta
from utils.hashutils import params_hash
from drawbot_driver import DrawbotDriver, DrawbotKinematics

MIN_US_A = 1952.25
MIN_ANGLE_RAD_A = pi/4.0
MAX_US_A = 1018.75
MAX_ANGLE_RAD_A = 3.0 * pi / 4.0
B_OFFSET_US = -39.0

PEN_UP_US = 1310.0
PEN_DOWN_US = 1500.0
PEN_UPDOWN_CHAN = 2
COUNTS_PER_US = 4
MIN_ANGLE_RAD_B = MIN_ANGLE_RAD_A
MAX_ANGLE_RAD_B = MAX_ANGLE_RAD_A
MIN_US_B = MIN_US_A + B_OFFSET_US
MAX_US_B = MAX_US_A + B_OFFSET_US

# Cal A 135=3*pi/4.0=1018.75us
#     120=1151.75
#     90=pi/2=1481.75
#     60=pi/6=1797.50
#     45=pi/4=1952.25
# B = offset = -39us = -39*4.0 = -146 counts
def angle_to_count(angle_rad, min_us, min_angle_rad, max_us, max_angle_rad):
    slope = (float(max_us) - float(min_us)) / (max_angle_rad - min_angle_rad)
    microsecs = min_us + ((angle_rad - min_angle_rad) * slope)
    counts = int(microsecs * 4.0)
    return counts


def find_highest_y(candidates):
    highest_y = candidates[0][1]
    idx = 0
    for cand_idx, p in enumerate(candidates):
        x, y = p
        if y > highest_y:
            highest_y = y
            idx = cand_idx

    return candidates[idx]


class CheapDrawBotKinematics(DrawbotKinematics):
    def __init__(self, *args, **kwargs):
        super(CheapDrawBotKinematics, self).__init__(*args, **kwargs)
        self.pa = kwargs.get("pa", (-65.0, 0.0))
        self.pb = kwargs.get("pb", (65.0, 0.0))
        self.ra = kwargs.get("ra", 40.0)
        self.rb = kwargs.get("rb", 40.0)
        self.la = kwargs.get("la", 95.0)
        self.lb = kwargs.get("lb", 95.0)
        # ext_co: extension coincident with la
        # ext_orth: extension orthogogonal to tip of ext_co
        self.ext_co = kwargs.get("ext_co", 18.0)
        self.ext_orth = kwargs.get("ext_orth", 9.5)

        # When la/lb doing "the splits" at the bottom, how far from the very bottom do we allow
        self.min_y_dist_from_bottom_line = kwargs.get("min_y_dist_from_bottom_line", 5.0)

        # Min/max angles achievable by servos (referenced to pa/pa)
        self.min_theta = kwargs.get("min_theta", pi / 16.0)
        self.max_theta = kwargs.get("max_theta", 15.0 * pi / 16.0)

        self.load_work_area_config()

    def get_kine_hash(self):
        return params_hash([self.pa[0], self.pa[1], self.pb[0], self.pb[1], self.ra, self.rb, self.la, self.lb, self.ext_co, self.ext_orth,
                            self.min_y_dist_from_bottom_line, self.min_theta, self.max_theta])

    def respects_external_constraints(self, endpoint, thetas):
        pex, pey = tuple(endpoint)
        theta1, theta2 = tuple(thetas)

        pax, pay = self.pa
        pbx, pby = self.pb
        if pey < (pay + self.ra + self.ext_orth + self.min_y_dist_from_bottom_line):
            return False

        if pey < (pby + self.rb + self.ext_orth + self.min_y_dist_from_bottom_line):
            return False

        if theta1 < self.min_theta or theta1 > self.max_theta:
            return False

        if theta2 < self.min_theta or theta1 > self.max_theta:
            return False

        return True

    def forward_kine(self, thetas, **kwargs):
        theta1, theta2 = tuple(thetas)

        lp = sqrt((self.la + self.ext_co) ** 2.0 + self.ext_orth ** 2.0)
        alpha = atan2(self.ext_orth, (self.la + self.ext_co))
        pax, pay = self.pa
        pbx, pby = self.pb

        # Point at angle theta1 on periphery of circle A
        cirax = pax + (self.ra * cos(theta1))
        ciray = pay + (self.ra * sin(theta1))

        # Point at angle theta2 on periphery of circle B
        cirbx = pbx + (self.rb * cos(theta2))
        cirby = pby + (self.rb * sin(theta2))

        # Find intersection of cirles of linkage la and lb
        c_candidates = get_intersection_2circ((cirax, ciray), self.la, (cirbx, cirby), self.lb)
        cx, cy = find_highest_y(c_candidates)

        # Find angles from linkages ra/rb tips to linkage la/lb tips
        phi1 = atan2((cy - ciray), (cx - cirax))
        phi2 = atan2((cy - cirby), (cx - cirbx))

        # Find offset pen position
        phi_p = phi1 + alpha

        # Find pen position
        pex = cirax + (lp * cos(phi_p))
        pey = ciray + (lp * sin(phi_p))

        if not self.respects_external_constraints((pex, pey), (theta1, theta2)):
            raise ValueError("No solution found for forward kine")

        return (pex, pey, phi1, phi2, cx, cy)

    def inverse_kine(self, end_point, **kwargs):
        # pe: Point of end effector (x, y) for which to find the theta1/theta2

        # ext_co: extension coincident with la
        # ext_orth: extension orthogogonal to tip of ext_co
        lp = sqrt((self.la + self.ext_co) ** 2.0 + self.ext_orth ** 2.0)
        alpha = atan2(self.ext_orth, (self.la + self.ext_co))
        pax, pay = self.pa
        pbx, pby = self.pb
        pex, pey = end_point

        # Find intersection between circle centered at `pe` of radius `lp`
        # and circle A. Keep highest y
        cira_candidates = get_intersection_2circ((pax, pay), self.ra, (pex, pey), lp)
        cirax, ciray = find_highest_y(cira_candidates)

        # Find linkage la/lb intersection point C from edge of circle a,
        # on linkage la, offset alpha from the lp linkage angle
        lp_angle = atan2((pey - ciray), (pex - cirax))
        c_angle = lp_angle - alpha
        cx = cirax + (self.la * cos(c_angle))
        cy = ciray + (self.la * sin(c_angle))

        # Find two intersecting points from circle centered at la/lb intersection
        # point C of radius lb, to circle B. Keep highest y
        cirb_candidates = get_intersection_2circ((cx, cy), self.lb, (pbx, pby), self.rb)
        cirbx, cirby = find_highest_y(cirb_candidates)

        # From points on cira/cirb on circle A/B, find linkage la/lb angles
        # theta1/theta2
        theta1 = atan2((ciray - pay), (cirax - pax))
        theta2 = atan2((cirby - pby), (cirbx - pbx))

        if self.respects_external_constraints((pex, pey), (theta1, theta2)):
            return (theta1, theta2, cx, cy)
        else:
            raise ValueError("No reverse kine solution found")

    def gen_path(self, points, seg_len=0.2):
        segment_points = []
        prev_px, prev_py = points[0]
        for px, py in points:
            # Split segments that are longer than maximum segment length into smaller chunks
            if distance((prev_px, prev_py), (px, py)) > seg_len:
                segment_points.extend(split_line_into_segments((prev_px, prev_py), (px, py), seg_len))
            else:
                segment_points.append((px, py))

            prev_px, prev_py = (px, py)

        all_angles = []
        for x, y in segment_points:
            try:
                # Find IK
                pe = (x, y)
                theta1, theta2, cx, cy = self.inverse_kine(pe)
                all_angles.append((theta1, theta2))
            except:
                pass

        return np.asarray(segment_points, dtype="float64"), np.asarray(all_angles, dtype="float64")

    def get_work_area(self, min_angle=pi / 16.0, max_angle=15.0 * pi / 16.0, ang_resolution=0.05, xy_resolution=0.5,
                      **kwargs):
        # Use cached data if available
        if "work_path" in self.work_area_config:
            return np.asarray(self.work_area_config["work_points"]), np.asarray(self.work_area_config["work_natives"]),\
                   np.asarray(self.work_area_config["work_path"])

        work_natives = []
        work_points = []

        # Coarse angle scan to find extreme X/Y for finer X/Y scan
        for theta1 in arange(max_angle, min_angle, -ang_resolution):
            for theta2 in arange(min_angle, max_angle, ang_resolution):
                try:
                    pex, pey, phi1, phi2, cx, cy = self.forward_kine((theta1, theta2))
                    work_natives.append((theta1, theta2))
                    work_points.append((pex, pey))
                except:
                    # No solution founds
                    pass

        min_x = 1e6
        max_x = -1e6
        min_y = 1e6
        max_y = -1e6

        for x, y in work_points:
            if x < min_x:
                min_x = x
            elif x > max_x:
                max_x = x

            if y < min_y:
                min_y = y
            elif y > max_y:
                max_y = y

        work_natives = []
        work_points = []

        for pey in arange(min_y, max_y, xy_resolution):
            for pex in arange(min_x, max_x, xy_resolution):
                try:
                    theta1, theta2, cx, cy = self.inverse_kine((pex, pey))
                    work_natives.append((theta1, theta2))
                    work_points.append((pex, pey))
                except:
                    # No solution founds
                    pass

        wheel_radius = xy_resolution * 2.0
        work_path, circle_centers = concave_hull_wheel(work_points, wheel_radius)
        self.work_area_config["work_path"] = work_path
        cx, cy, width, height, min_x, max_x, min_y, max_y = centroid_extents(work_path[:-1])
        self.work_area_config["work_centroid"] = [cx, cy]

        # Save HPGL of feasible area
        if kwargs.get("save_hpgl") is not None:
            left_bottom_mm = (0.0, 0.0)
            right_top_mm = (2.0 * self.pb[0], 1.25 * max_y)
            path_array_mm = []

            for pex, pey in work_points:
                path_array_mm.append((pex - self.pa[0], pey))

            filename = kwargs.get("save_hpgl")
            save_path_as_hpgl(path_array_mm, left_bottom_mm, right_top_mm, filename)

        # Plot feasible area if requested
        if kwargs.get("plot", False):
            axis_limits = [self.pa[0], self.pb[0], 0, 1.25 * max_y] # [xmin, xmax, ymin, ymax]
            plot_feasible_xy_theta(feas_path_points=work_points, feas_thetas=work_natives, axis_limits=axis_limits)

        # Save computation in cache if required
        if kwargs.get("save_cache", False):
            self.save_work_area_config()

        return np.asarray(work_points), np.asarray(work_natives), np.asarray(work_path)

    def draw_robot_preview(self, ax, show_robot=False, show_work_area=True, **kwargs):
        # If work area was never computed, compute it now
        if show_work_area:
            if self.work_area_config.get("work_path") is None:
                xy_resolution = 0.5
                _, _, work_path = self.get_work_area(xy_resolution=xy_resolution, save_cache=True)
            else:
                work_path = np.asarray(self.work_area_config["work_path"])

            ax.plot(work_path[:, 0], work_path[:, 1], "r-")

        #for idx, cir in enumerate(circle_centers):
        #    c = plt.Circle((cir[0], cir[1]), wheel_radius, fill=False, color='g')
        #    t = plt.Text(x=cir[0], y=cir[1], text="%d" % idx)
        #    ax.add_artist(c)
        #    ax.add_artist(t)


class CheapDrawbot(DrawbotDriver):
    def __init__(self, drawbot_kinematics, *args, **kwargs):
        super(CheapDrawbot, self).__init__(drawbot_kinematics, *args, **kwargs)
        self.serial_device = kwargs.get("serial_device", "COM6")
        self.maestro = None        
        
    def connect_impl(self):
        self.maestro = pololu_maestro.PololuMaestro(self.serial_device)
        self.maestro.connect()
        
    def disconnect_impl(self):
        if self.maestro is not None:
            self.maestro.close()

    def set_pen_height_impl(self, height_mm):
        if height_mm > 0.1:
            self.maestro.set_target(PEN_UPDOWN_CHAN, int(PEN_UP_US * COUNTS_PER_US))
        else:
            self.maestro.set_target(PEN_UPDOWN_CHAN, int(PEN_DOWN_US * COUNTS_PER_US))
        time.sleep(0.3)

    def set_natives_impl(self, natives):
        theta1 = natives[0]
        theta2 = natives[1]
        
        counts1 = angle_to_count(theta1, MIN_US_A, MIN_ANGLE_RAD_A, MAX_US_A, MAX_ANGLE_RAD_A)
        counts2 = angle_to_count(theta2, MIN_US_B, MIN_ANGLE_RAD_B, MAX_US_B, MAX_ANGLE_RAD_B)

        if self.maestro is not None:
            self.maestro.set_target(0, counts1)
            self.maestro.set_target(1, counts2)


def build_cheap_drawbot(**kwargs):
    drawbot_kinematics = CheapDrawBotKinematics(**kwargs)
    drawbot = CheapDrawbot(drawbot_kinematics, **kwargs)

    return drawbot
