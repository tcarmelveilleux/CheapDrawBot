#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""

Author: Tennessee Carmel-Veilleux
Created on: 2017-03-10
"""
from chiplotle.tools.serialtools.virtual_serial_port import VirtualSerialPort
from chiplotle.plotters.plotter import Plotter
from chiplotle.geometry.core.coordinate import Coordinate
from chiplotle.tools.io import view
from chiplotle.tools.io import import_hpgl_file, save_hpgl, export
from chiplotle.geometry.shapes import label, line, group
from chiplotle.tools.hpgltools import *
from chiplotle import *

def main():
    #hpgl = import_hpgl_file("test1.hpgl")
    #for cmd in hpgl:
    #    print cmd

    left_bottom = (-1016,-1016)
    right_top = (1016,1016)
    plotter = Plotter(VirtualSerialPort(left_bottom=left_bottom, right_top=right_top))

    plotter.pen_up()
    plotter.goto(100,100)
    plotter.pen_down()

    plotter.pen_down()
    l = shapes.label("Roboto", 2, 2)
    plotter.write(l)
    plotter.write(shapes.circle(500))

    l1 = shapes.line((0,0), (-1000,-1000))
    l2 = shapes.line((0, -1000), (-1000, 0))
    plotter.write(l1)
    plotter.write(l2)


    #view(plotter, fmt="pcx")

    export(plotter, "test1.pcx", fmt="pcx")

if __name__ == '__main__':
    main()
