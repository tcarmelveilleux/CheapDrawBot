#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
HPGL Loading Activity

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""
import numpy as np
from chiplotle.tools.io import import_hpgl_file
from chiplotle.hpgl.commands import PD, PU
from activity import Activity

class DrawHpglActivity(Activity):
    def __init__(self, parent, drawbot, *args, **kwargs):
        super(DrawHpglActivity, self).__init__(parent, drawbot, *args, **kwargs)
        self._name = "HPGL"
        self._param_ctrls = {}
        self._params = {}
        self._paths = []
        self._program = []
        self._filename = "Funstuff.hpgl"
        self._parent.handle_event({"event": "activity_updated"})

    def handle_event(self, event_dict):
        # If parent handled event, return
        if super(DrawHpglActivity, self).handle_event(event_dict):
            return True

        #event_type = event_dict["event"]
        #if event_type == "param_changed":
        #    self._parent.handle_event({"event": "activity_updated"})
        #    return True
        return False

    def update_geometry(self):
        hpgl_data = import_hpgl_file(self._filename)

        prev_x = 0
        prev_y = 0
        pen_is_up = False

        accumulated_paths = []
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

    def draw_preview(self, ax):
        xmin = min(self._x)
        xmax = max(self._x)
        ymin = min(self._y)
        ymax = max(self._y)

        drawing_path = np.column_stack((self._x, self._y))

        extents = [xmin, xmax, ymin, ymax]

        ax.plot(drawing_path[:, 0], drawing_path[:, 1], 'b-')
        #ax.axis(extents)

    def start_drawing(self):
        self._drawbot.pen_up()
        self._drawbot.goto((self._x[0], self._y[0]))
        self._drawbot.pen_down()
        drawing_path = np.column_stack((self._x, self._y))
        self._drawbot.draw_path(drawing_path)
        self._drawbot.pen_up()
