#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Manual Controller activity

Author: tennessee
Created on: 2017-04-22

Copyright 2017, Tennessee Carmel-Veilleux.
"""
import numpy as np
from activity import Activity, NumericalActivityParam

class ManualControlActivity(Activity):
    def __init__(self, parent, drawbot, *args, **kwargs):
        super(ManualControlActivity, self).__init__(parent, drawbot, *args, **kwargs)
        self._name = "ManualControl"
        self._param_ctrls = {}
        self._params = {
            "x": NumericalActivityParam(name="x", desc="X (mm)", value=0.5, fmt="%.3f", min_val=0.01, max_val=0.99),
            "y": NumericalActivityParam(name="y", desc="Y (mm)", value=0.35, fmt="%.3f", min_val=0.01, max_val=0.99),
            "z": NumericalActivityParam(name="z", desc="Z (mm)", value=30.0, fmt="%.1f", min_val=10.0, max_val=30.0),
        }
        self._x = []
        self._y = []
        self.update_geometry()

    def handle_event(self, event_dict):
        # If parent handled event, return
        if super(ManualControlActivity, self).handle_event(event_dict):
            return True

        event_type = event_dict["event"]
        if event_type == "param_changed":
            self._parent.handle_event({"event": "activity_updated"})
            return True
        elif event_type == "hmi_event":
            hmi_event = event_dict["data"]
            axis = hmi_event["axis"]
            if axis == "axis0":
                self._drawbot.pen_up(height_mm=3.0 * hmi_event["value"])

            return True

        return False

    def update_geometry(self):
        pass

    def draw_preview(self, ax):
        return

        """
        xmin = min(self._x)
        xmax = max(self._x)
        ymin = min(self._y)
        ymax = max(self._y)

        drawing_path = np.column_stack((self._x, self._y))

        extents = [xmin, xmax, ymin, ymax]

        ax.plot(drawing_path[:, 0], drawing_path[:, 1], 'b-')
        #ax.axis(extents)
        """

    def start_drawing(self):
        self._drawbot.pen_up()
        self._drawbot.goto((self._x[0], self._y[0]))
        self._drawbot.pen_down()
        drawing_path = np.column_stack((self._x, self._y))
        self._drawbot.draw_path(drawing_path)
        self._drawbot.pen_up()
