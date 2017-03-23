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
from numpy import pi
from utils.geometry import *
from utils.hpglutils import save_path_as_hpgl
from utils.plotutils import plot_feasible_xy_theta
from numpy import arange
from drawbot_driver import RobotDriver, RobotKinematics

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


def pen_up(maestro):
    maestro.set_target(PEN_UPDOWN_CHAN, int(PEN_UP_US * COUNTS_PER_US))
    time.sleep(0.3)


def pen_down(maestro):
    maestro.set_target(PEN_UPDOWN_CHAN, int(PEN_DOWN_US * COUNTS_PER_US))
    time.sleep(0.3)


class CheapDrawBotKinematics(RobotKinematics):
    def __init__(self, *args, **kwargs):
        super(CheapDrawBotKinematics, self).__init__(*args, **kwargs)
        self.pa = (-65.0, 0.0)
        self.pb = (65.0, 0.0)
        self.ra = 40.0
        self.rb = 40.0
        self.la = 95.0
        self.lb = 95.0
        # ext_co: extension coincident with la
        # ext_orth: extension orthogogonal to tip of ext_co
        self.ext_co = 18.0
        self.ext_orth = 9.5

        # When la/lb doing "the splits" at the bottom, how far from the very bottom do we allow
        self.min_y_dist_from_bottom_line = 5.0

        # Min/max angles achievable by servos (referenced to pa/pa)
        self.min_theta = pi / 16.0
        self.max_theta = 15.0 * pi / 16.0

    def respects_external_constraints(self, pex, pey, theta1, theta2):
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

    def forward_kine(self, theta1, theta2):
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

        if not self.respects_external_constraints(pex, pey, theta1, theta2):
            raise ValueError("No solution found for forward kine")

        return (pex, pey, phi1, phi2, cx, cy)

    def inverse_kine(self, pe):
        # pe: Point of end effector (x, y) for which to find the theta1/theta2

        # ext_co: extension coincident with la
        # ext_orth: extension orthogogonal to tip of ext_co
        lp = sqrt((self.la + self.ext_co) ** 2.0 + self.ext_orth ** 2.0)
        alpha = atan2(self.ext_orth, (self.la + self.ext_co))
        pax, pay = self.pa
        pbx, pby = self.pb
        pex, pey = pe

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

        if self.respects_external_constraints(pex, pey, theta1, theta2):
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
                # Find ik
                pe = (x, y)
                theta1, theta2, cx, cy = self.inverse_kine(pe)
                all_angles.append((theta1, theta2))
            except:
                pass

        return segment_points, all_angles

    def local_coord_from_hgpl(self, p):
        return ((p[0] * 25e-3) + self.pa[0], (p[1] * 25e-3) + self.pa[1])

    def get_feasible_area(self, min_angle=pi / 16.0, max_angle=15.0 * pi / 16.0, ang_resolution=0.05, xy_resolution=0.5,
                          **kwargs):
        feas_thetas = []
        feas_points = []

        # Coarse angle scan to find extreme X/Y for finer X/Y scan
        for theta1 in arange(max_angle, min_angle, -ang_resolution):
            for theta2 in arange(min_angle, max_angle, ang_resolution):
                try:
                    pex, pey, phi1, phi2, cx, cy = self.forward_kine(theta1, theta2)
                    feas_thetas.append((theta1, theta2))
                    feas_points.append((pex, pey))
                except:
                    # No solution founds
                    pass

        min_x = 1e6
        max_x = -1e6
        min_y = 1e6
        max_y = -1e6

        for x, y in feas_points:
            if x < min_x:
                min_x = x
            elif x > max_x:
                max_x = x

            if y < min_y:
                min_y = y
            elif y > max_y:
                max_y = y

        feas_thetas = []
        feas_points = []

        for pey in arange(min_y, max_y, xy_resolution):
            for pex in arange(min_x, max_x, xy_resolution):
                try:
                    theta1, theta2, cx, cy = self.inverse_kine((pex, pey))
                    feas_thetas.append((theta1, theta2))
                    feas_points.append((pex, pey))
                except:
                    # No solution founds
                    pass

        # Save HPGL of feasible area
        if kwargs.get("save_hpgl") is not None:
            left_bottom_mm = (0.0, 0.0)
            right_top_mm = (2.0 * self.pb[0], 1.25 * max_y)
            path_array_mm = []

            for pex, pey in feas_points:
                path_array_mm.append((pex - self.pa[0], pey))

            filename = kwargs.get("save_hpgl")
            save_path_as_hpgl(path_array_mm, left_bottom_mm, right_top_mm, filename)
        
        # Plot feasible area if requested
        if kwargs.get("plot", False):
            axis_limits = [self.pa[0], self.pb[0], 0, 1.25 * max_y] # [xmin, xmax, ymin, ymax]
            plot_feasible_xy_theta(feas_path_points=feas_points, feas_thetas=feas_thetas, axis_limits=axis_limits)

        return feas_points, feas_thetas
