#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Random geometry utilities useful for kinematics and interpolation.

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""
from __future__ import print_function
from numpy import sqrt, pi, sin, cos, floor
from numpy import arctan2 as atan2
import numpy as np
from collections import Sequence

def get_intersection_2circ(p0, r0, p1, r1):
    p0x, p0y = tuple(p0)
    p1x, p1y = tuple(p1)

    d = sqrt(((p0x - p1x) ** 2 + (p0y - p1y) ** 2))
    if d > (r0 + r1):
        # Separate circles
        return []
    elif d < abs(r0 - r1):
        # Concentric circles
        return []
    elif d == 0 and r0 == r1:
        # Coincident circles
        return []

    a = ((r0 ** 2) - (r1 ** 2) + (d ** 2)) / (2.0 * d)

    h = sqrt((r0 ** 2) - (a ** 2))

    p2x = p0x + (a * (p1x - p0x)) / d
    p2y = p0y + (a * (p1y - p0y)) / d

    p3x_1 = p2x + (h * (p1y - p0y)) / d
    p3x_2 = p2x - (h * (p1y - p0y)) / d

    p3y_1 = p2y - (h * (p1x - p0x)) / d
    p3y_2 = p2y + (h * (p1x - p0x)) / d

    return ((p3x_1, p3y_1), (p3x_2, p3y_2))


def get_circ_center_2pts_r(p1, p2, r):
    """
    Find the centers of the two circles that share two points p1/p2 and a radius.

    From algorithm at http://mathforum.org/library/drmath/view/53027.html. Adapted from version at
    https://rosettacode.org/wiki/Circles_of_given_radius_through_two_points#Python.

    :param p1: First point , tuple (x, y)
    :param p2: Second point, tuple (x, y)
    :param r: Radius of circle
    :return: a list of 2 points that are centers of circles of radius r sharing p1/p2
    """
    if r == 0.0:
        raise ValueError('No solution due to no radius')

    (x1, y1), (x2, y2) = tuple(p1), tuple(p2)

    if p1 == p2:
        raise ValueError('Infinite numbre of solutions')

    # Distance in x and y between points
    dx = x2 - x1
    dy = y1 - y2

    # Dist between points
    q = sqrt(dx ** 2 + dy ** 2)

    if q > (2.0 * r):
        raise ValueError('Too much distance between points to fit within radius')

    # Halfway point
    x3 = (x1 + x2) / 2.0
    y3 = (y1 + y2) / 2.0

    # Distance along the mirror line
    d = sqrt(r ** 2 - ((q / 2.0) ** 2))

    # First circle center
    # c1 = (x3 + ((d * dy) / q), y3 + ((d * dx) / q))

    # Second circle center
    # c2 = (x3 - ((d * dy) / q), y3 - ((d * dx) / q))

    c1x = x3 + sqrt(r ** 2 - (q / 2.0) ** 2) * (y1 - y2) / q
    c1y = y3 + sqrt(r ** 2 - (q / 2.0) ** 2) * (x2 - x1) / q

    c2x = x3 - sqrt(r ** 2 - (q / 2.0) ** 2) * (y1 - y2) / q
    c2y = y3 - sqrt(r ** 2 - (q / 2.0) ** 2) * (x2 - x1) / q

    return ((c1x, c1y), (c2x, c2y))


def centroid_extents(all_xy):
    """
    Find the centroid and bounding box of a list of cloud of points

    :param all_xy: List of arrays Nx2 (x,y) pairs

    :return: tuple (cx, cy, width, height, min_x, max_x, min_y, max_y) of centroid/bounding box
    """
    if len(all_xy) == 0:
        raise ValueError("Cannot find extents of nothing !")

    if not isinstance(all_xy, Sequence):
        all_xy = [all_xy]

    if len(all_xy[0].shape) == 1:
        min_x = all_xy[0][0]
        max_x = all_xy[0][0]
        min_y = all_xy[0][1]
        max_y = all_xy[0][1]
    else:
        min_x = all_xy[0][0,0]
        max_x = all_xy[0][0,0]
        min_y = all_xy[0][0,1]
        max_y = all_xy[0][0,1]

    cx = 0.0
    cy = 0.0
    num_x = 0
    num_y = 0

    for group in all_xy:
        if isinstance(group, Sequence):
            group = np.asarray(group, float)

        if len(group.shape) == 1:
            # Handle input being a list of one-dimensional, 2-element arrays
            min_x = min(min_x, group[0])
            max_x = max(max_x, group[0])
            min_y = min(min_y, group[1])
            max_y = max(max_y, group[1])

            cx += group[0]
            cy += group[1]
            num_x += 1
            num_y += 1
        else:
            min_x = min(min_x, np.amin(group[:, 0]))
            max_x = max(max_x, np.amax(group[:, 0]))
            min_y = min(min_y, np.amin(group[:, 1]))
            max_y = max(max_y, np.amax(group[:, 1]))

            cx += np.sum(group[:, 0])
            cy += np.sum(group[:, 1])
            num_x += np.alen(group[:, 0])
            num_y += np.alen(group[:, 1])

    cx /= num_x
    cy /= num_y
    width = max_x - min_x
    height = max_y - min_y

    return (cx, cy, width, height, min_x, max_x, min_y, max_y)


def distance(p1, p2):
    """
    Distance between two points p1 and p2

    :param p1: First point
    :param p2: Second point
    :return: The euclidean distance between the two points
    """
    return sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def dotp2(v1, v2):
    return (v1[0] * v2[0]) + (v1[1] * v2[1])


def crossp2(v1, v2):
    return (v1[0] * v2[1]) - (v1[1] * v2[0])


def angle_between_signed(v1, v2):
    return atan2(crossp2(v1, v2), dotp2(v1, v2))


def find_closest_point(all_xy, p):
    min_idx = 0
    min_dist = distance(all_xy[0], p)

    for idx, (x, y) in enumerate(all_xy):
        dist = distance((x, y), p)
        if dist < min_dist:
            min_idx = idx
            min_dist = dist

    return min_idx, min_dist, all_xy[min_idx]


def circle_empty(all_xy, center, radius):
    eps = radius / 1e6
    for point in all_xy:
        # Is current point within circle ? If so, circle not empty. Circumference is excluded
        if distance(point, center) < (radius - eps):
            return False
    return True


def sorted_by_manhattan_distance(all_xy, p, within):
    candidates = [p2 for p2 in all_xy if (abs(p2[0] - p[0]) + abs(p2[1] - p[1])) <= within]
    return sorted(candidates, key=lambda p2: (abs(p2[0] - p[0]) + abs(p2[1] - p[1])))


def concave_hull_wheel(all_xy, wheel_radius):
    # Get centroid and bounding box of points
    cx, cy, width, height, min_x, max_y, min_y, max_y = centroid_extents(all_xy)

    # Offset all points relative to the centroid
    centered_all_xy = []
    for x, y in all_xy:
        centered_all_xy.append((x - cx, y - cy))

    # Determine starting segment, which is outside shape (for sure)
    r = 2.0 * max(width, height)
    theta = 0
    start_point = (r * cos(theta), r * sin(theta))

    # Find point closest to tip of segment at 0, it becomes our actual starting point
    min_idx, min_dist, _ = find_closest_point(centered_all_xy, start_point)
    start_point = tuple(all_xy[min_idx])

    print("Starting point: %s" % repr(start_point))

    outline_points = [start_point]
    circle_centers = []
    last_point = start_point

    done = False
    while not done:
        # Find next closest points of a rolling circle where the circle advances clockwise (negative cross product)
        min_idx = 0
        min_dist = r
        min_circle_center = (0, 0)
        found = False

        candidates = sorted_by_manhattan_distance(all_xy, last_point, within=(4.0 * wheel_radius))
        for idx, candidate in enumerate(candidates):
            candidate = tuple(candidate)

            # Skip last point (starting point of current circle)
            if candidate == last_point:
                continue

            dist = distance(last_point, candidate)
            # Point too far to fit on circle edge
            if dist > (2.0 * wheel_radius):
                continue

            try:
                cir_candidates = get_circ_center_2pts_r(last_point, candidate, wheel_radius)
            except ValueError:
                # Cannot find two circles
                continue

            # Try both candidates to find a circle that is empty
            empty_circle_center = None
            num_empty = 0
            for circle_center in cir_candidates:
                if circle_empty(candidates, circle_center, wheel_radius):
                    num_empty += 1
                    empty_circle_center = circle_center

            if num_empty == 0:
                # No empty circles: not a circle rolling on the outside
                # print("Found no empty: %s" % repr(cir_candidates))
                continue
            elif num_empty == 2:
                # Should never see two emtpy circles
                # print("Found two empty circle candidates for last point %s: %s" % (last_point, cir_candidates))
                continue

            # Find rotation of "wheel" by tracing vector v1 from center of circle to last point, and v2 from center
            # of circle to candidate. If angle is negative, wheel is rotating counter-clockwise around points.
            # Imagine trying to see if a "spoke" on a wheel turned clockwise while rotating counter-clockwise around
            # a circle.
            v1 = (last_point[0] - empty_circle_center[0], last_point[1] - empty_circle_center[1])
            v2 = (candidate[0] - empty_circle_center[0], candidate[1] - empty_circle_center[1])

            if angle_between_signed(v1, v2) >= 0:
                continue

            # We got here, found a candidate point! Determine if it is the minimum!
            if dist < min_dist:
                # print("Found a new minimum candidate: %s", repr(all_xy[min_idx]))
                min_dist = dist
                min_idx = idx
                min_circle_center = empty_circle_center
                found = True
                # break

        if not found:
            raise ValueError("Computation diverged and no candidate was found. Should not happen!")

        # Got through all the points. New outline point is on concave hull
        outline_point = tuple(candidates[min_idx])
        print("Found outline point (%.2f, %.2f)" % (outline_point[0], outline_point[1]))
        if outline_point in outline_points:
            # Got back to start, we're done!
            done = True
            # Append starting point again to close the path
            outline_points.append(outline_point)
        else:
            circle_centers.append(min_circle_center)
            outline_points.append(outline_point)
            last_point = outline_point

    return np.asarray(outline_points, dtype="float64"), np.asarray(circle_centers, dtype="float64")


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
        segments.append((x, y))

    segments.append((p2x, p2y))

    return segments


def get_scale_2d(scale):
    """
    Generate a 2D homogenous coordinates transformation matrix for scaling

    :param scale: a scalar of the scale factor to apply: < 1.0 shrinks, > 1.0 grows, 1.0 is identity
    :return: A 3x3 transformation matrix applying the given scaling
    """
    return np.array([[scale, 0.0, 0.0],
                     [0.0, scale, 0.0],
                     [0.0, 0.0, 1.0]])

def get_translation_2d(translation):
    """
    Generate a 2D homogenous coordinates transformation matrix for a translation

    :param translation: (x,y) pair of translation coordinates
    :return: A 3x3 transformation matrix applying the given translation
    """
    return np.array([[1.0, 0.0, translation[0]],
                     [0.0, 1.0, translation[1]],
                     [0.0, 0.0, 1.0]])

def get_rotation_2d(theta, around=(0.0, 0.0)):
    """
    Generate a 2D homogenous coordinates transformation matrix for a rotation around a point

    :param theta: Rotation angle in radians (positive == CCW)
    :param around: Point around which to affect the center of rotation (default (0, 0))
    :return: A 3x3 transformation matrix applying the given rotation
    """
    return np.array([[cos(theta), -sin(theta), around[0] * (1.0 - cos(theta)) + (around[1] * sin(theta))],
                     [sin(theta), cos(theta), around[1] * (1.0 - cos(theta)) - (around[0] * sin(theta))],
                     [0.0, 0.0, 1.0]])

def apply_transform_2d(points, transform):
    """
    Apply a 2D tranformation matrix (3x3 with input points as column vectors convention) to an array of 2D points.
    2D points must be an array of 2D row vectors: lines are the points (x,y).

    :param points: Array of row vectors of 2D points to transform
    :param transform: 3x3 transformation matrix to apply
    :return: Transformed array of row vectors
    """
    # Augment with a column of 1s, transport to column vectors of points
    augmented_points = np.column_stack((points, np.ones((points.shape[0], 1)))).transpose()

    # 3x3 * 3xn = 3xn -> transpose to row vectors of points, remove augmentation
    transformed = np.dot(transform, augmented_points).transpose()[:, 0:2]

    return transformed

def list_of_tuples(a_2d_array):
    return [tuple(row) for row in a_2d_array]