#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Spirograph Activity

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""
import numpy as np
from activity import Activity, NumericalActivityParam

class SpirographActivity(Activity):
    def __init__(self, parent, drawbot, *args, **kwargs):
        super(SpirographActivity, self).__init__(parent, drawbot, *args, **kwargs)
        self._name = "Spirograph"
        self._param_ctrls = {}
        self._params = {
            "turns": NumericalActivityParam(name="turns", desc="Num Turns", value=6.0, fmt="%.1f", min_val=1.0, max_val=10.0),
            "l": NumericalActivityParam(name="l", desc="l", value=0.5, fmt="%.3f", min_val=0.01, max_val=0.99),
            "k": NumericalActivityParam(name="k", desc="k", value=0.35, fmt="%.3f", min_val=0.01, max_val=0.99),
            "R": NumericalActivityParam(name="R", desc="R", value=30.0, fmt="%.1f", min_val=10.0, max_val=30.0),
        }
        self._x = []
        self._y = []
        self.update_geometry()

    def handle_event(self, event_dict):
        # If parent handled event, return
        if super(SpirographActivity, self).handle_event(event_dict):
            return

        event_type = event_dict["event"]
        if event_type == "param_changed":
            self._parent.handle_event({"event": "activity_updated"})

    def update_geometry(self):
        n_turns = self._params["turns"].value
        R = self._params["R"].value
        l = self._params["l"].value
        k = self._params["k"].value

        quality = 4
        scale = 2 * np.pi / (60.0 * quality)
        max_points = n_turns * 2.0 * np.pi * n_turns / scale

        theta = np.arange(0, max_points) * scale

        tx, ty = tuple(self._drawbot.kine.get_work_area_centroid())

        self._x = tx + R * ((1 - k) * np.cos(theta) + (l * k) * np.cos(((1 - k) / k) * theta))
        self._y = ty + R * ((1 - k) * np.sin(theta) - (l * k) * np.sin(((1 - k) / k) * theta))

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
