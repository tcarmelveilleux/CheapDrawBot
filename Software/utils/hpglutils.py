#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
HPGL utilities

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux
"""

from chiplotle.tools.serialtools.virtual_serial_port import VirtualSerialPort
from chiplotle.plotters.plotter import Plotter
from chiplotle.geometry.core.coordinate import Coordinate
from chiplotle.geometry.core.coordinatearray import CoordinateArray
from chiplotle.tools.io import view, import_hpgl_file, save_hpgl, export
from chiplotle.hpgl.commands import PD, PU


def save_path_as_hpgl(path_array_mm, left_bottom_mm, right_top_mm, filename):
    def local_mm_to_hpgl(x, y):
        return (int(x / 25e-3), int(y / 25e-3))

    hpgl_coords = CoordinateArray()
    plotter = Plotter(VirtualSerialPort(left_bottom=local_mm_to_hpgl(*left_bottom_mm),
                                        right_top=local_mm_to_hpgl(*right_top_mm)))
    for x, y in path_array_mm:
        hpgl_coords.append(Coordinate(*local_mm_to_hpgl(x, y)))

    plotter.pen_down(hpgl_coords)
    save_hpgl(plotter, filename)
