#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Fun Matplotlib figures related to robot geometry

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""
import matplotlib.pyplot as plt


def plot_feasible_xy_theta(feas_path_points, feas_thetas, axis_limits, fig=None):
    if fig is None:
        f = plt.figure()
    else:
        f = fig

    ax1 = f.add_subplot(121)
    ax1.plot([x for x, y in feas_path_points], [y for x, y in feas_path_points], 'r-')
    ax1.axis('equal')
    ax1.axis(axis_limits) #[-65, 65, 0, 150]
    ax1.title("Feasible X/Y area")

    ax2 = f.add_subplot(122)
    ax2.plot([t1 for t1, t2 in feas_thetas], [t2 for t1, t2 in feas_thetas], 'b.')
    ax2.axis('equal')
    ax2.title("Feasible theta space")

    # Show figure if no external figure provided
    if fig is None:
        plt.show()
