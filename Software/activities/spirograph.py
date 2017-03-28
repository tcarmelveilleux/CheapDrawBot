#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Spirograph Activity

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""
import numpy as np
from activity import Activity, ActivityParam

class SpirographActivity(Activity):
    def __init__(self, parent, drawbot, *args, **kwargs):
        super(SpirographActivity, self).__init__(parent, drawbot, *args, **kwargs)
        self._name = "Spirograph"
        self._param_ctrls = {}
        self._params = {
            "turns": ActivityParam(name="turns", desc="Num Turns", fmt="%.1f", min_val=1.0, max_val=10.0, value=6.0),
            "l": ActivityParam(name="l", desc="l", fmt="%.3f", min_val=0.01, max_val=0.99, value=0.5),
            "k": ActivityParam(name="k", desc="k", fmt="%.3f", min_val=0.01, max_val=0.99, value=0.35),
            "R": ActivityParam(name="R", desc="R", fmt="%.1f", min_val=10.0, max_val=30.0, value=20.0),
        }
        self._x = []
        self._y = []

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

        self._x = R * ((1 - k) * np.cos(theta) + (l * k) * np.cos(((1 - k) / k) * theta))
        self._y = R * ((1 - k) * np.sin(theta) - (l * k) * np.sin(((1 - k) / k) * theta))

    def draw_preview(self, ax):
        xmin = min(self._x)
        xmax = max(self._x)
        ymin = min(self._y)
        ymax = max(self._y)

        drawing_path = np.column_stack((self._x, self._y))

        extents = [xmin, xmax, ymin, ymax]

        ax.clear()
        ax.plot(drawing_path[:, 0], drawing_path[:, 1], 'b-')
        #ax.axis(extents)
