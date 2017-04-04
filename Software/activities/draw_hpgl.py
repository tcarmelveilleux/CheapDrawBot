#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
HPGL Loading Activity

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""
import numpy as np
import os
from collections import OrderedDict
from chiplotle.tools.io import import_hpgl_file
from chiplotle.hpgl.commands import PD, PU
from activity import Activity, FilenameActivityParam, ButtonActivityParam, NumericalActivityParam

class DrawHpglActivity(Activity):
    def __init__(self, parent, drawbot, *args, **kwargs):
        super(DrawHpglActivity, self).__init__(parent, drawbot, *args, **kwargs)
        self._name = "HPGL"
        self._filename = "Funstuff.hpgl"
        self._param_ctrls = {}
        self._params = OrderedDict(
            (("hpgl_filename", FilenameActivityParam(name="hpgl_filename", desc="HPGL file to plot",
                                                   value=self._filename, is_save=False, filetypes=[("HPGL Files", "*.hpgl")])),
            ("load_file", ButtonActivityParam(name="load_file", desc="Load HPGL file", value="load_file")),
            ("tx", NumericalActivityParam(name="tx", desc="X translation (mm)", value=0.0, fmt="%.1f", min_val=-100.0, max_val=100.0)),
            ("ty", NumericalActivityParam(name="ty", desc="Y translation (mm)", value=0.0, fmt="%.1f", min_val=-100.0, max_val=100.0)))
        )
        self._paths = []
        self._transformed_paths = []
        self._program = []

    def handle_event(self, event_dict):
        # If parent handled event, return
        if super(DrawHpglActivity, self).handle_event(event_dict):
            return True

        event_type = event_dict["event"]
        if event_type == "param_changed":
            param = event_dict["param"]
            # Handle the Load File button
            if param.name == "load_file":
                filename = self._params["hpgl_filename"].value
                if not os.path.isfile(filename):
                    message = "File with path:\n'%s'\ndoes not exist!" % filename
                    error_event = {"event": "error", "title": "File load error...", "message": message}
                    return self.handle_event(error_event)

                # TODO: Validate existence!
                self._filename = filename
                self._logger.info("Trying to load %s", filename)
                self._parent.handle_event({"event": "activity_updated"})
                return True
            elif param.name in ("tx", "ty"):
                # Update activity on translation change
                self._parent.handle_event({"event": "activity_updated"})

        return False

    def update_geometry(self):
        def hpgl2mm(x, y):
            return (x * 25e-3, y * 25e-3)

        try:
            hpgl_data = import_hpgl_file(self._filename)
        except BaseException as e:
            message = "Could not load '%s'\nException: %s, %s" % (self._filename, e.__class__.__name__, str(e))
            error_event = {"event": "error", "title": "Error loading HPGL file...",
                           "message": message}
            self.handle_event(error_event)
            return


        accumulated_paths = []
        current_path = []
        for command in hpgl_data:
            if isinstance(command, PU):
                if len(current_path) > 0:
                    accumulated_paths.append(np.asarray(current_path, dtype="float64"))
                    current_path = []

                current_path = [hpgl2mm(command.x[0], command.y[0])]
            elif isinstance(command, PD):
                for coord in command.xy:
                    current_path.append(hpgl2mm(coord.x, coord.y))

        # Flush last path if no pen up
        if len(current_path) > 0:
            accumulated_paths.append(np.asarray(current_path, dtype="float64"))

        self._paths = accumulated_paths

    # TODO: Replace with a global helper!
    def _update_transformed_paths(self):
        self._transformed_paths = []
        tx = self._params["tx"].value
        ty = self._params["ty"].value
        for path in self._paths:
            self._transformed_paths.append(path + np.asarray((tx, ty)))

    def draw_preview(self, ax):
        self._update_transformed_paths()

        nothing_to_show = False
        if len(self._transformed_paths) == 0 or len(self._transformed_paths[0]) == 0:
            xmin = 0
            xmax = 1
            ymin = 0
            ymax = 1
            nothing_to_show = True
        else:
            xmin = self._transformed_paths[0][0, 0]
            xmax = self._transformed_paths[0][0, 0]
            ymin = self._transformed_paths[0][0, 1]
            ymax = self._transformed_paths[0][0, 1]

        for path in self._transformed_paths:
            xmin = min(xmin, np.amin(path[:,0]))
            xmax = max(xmax, np.amax(path[:,0]))
            ymin = min(ymin, np.amin(path[:,1]))
            ymax = max(ymax, np.amax(path[:,1]))

            ax.plot(path[:, 0], path[:, 1], 'b-')

        extents = [xmin, xmax, ymin, ymax]

        #ax.axis(extents)

    def start_drawing(self):
        self._update_transformed_paths()

        for path in self._transformed_paths:
            # Ignore empty paths
            if len(path) == 0:
                continue

            # Move to starting point of current path
            self._drawbot.pen_up()
            self._drawbot.goto((path[0, 0], path[0, 1]))

            # Don't go down if up-point was only point
            if len(path) == 1:
                continue

            # Draw entire path
            self._drawbot.pen_down()
            self._drawbot.draw_path(path)

        # Lift pen at end
        self._drawbot.pen_up()
