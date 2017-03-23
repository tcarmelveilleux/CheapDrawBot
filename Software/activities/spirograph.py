#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Spirograph Activity

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""

from activity import Activity, ActivityParam

class SpirographActivity(Activity):
    def __init__(self, master, controller, *args, **kwargs):
        super(SpirographActivity, self).__init__(master, controller, *args, **kwargs)
        self._name = "Spirograph"
        self._param_ctrls = {}
        self._params = {
            "turns": ActivityParam(name="turns", desc="Num Turns", fmt="%.1f", min_val=1.0, max_val=10.0, value=6.0),
            "l": ActivityParam(name="l", desc="l", fmt="%.3f", min_val=0.01, max_val=0.99, value=0.5),
            "k": ActivityParam(name="k", desc="k", fmt="%.3f", min_val=0.01, max_val=0.99, value=0.35),
            "R": ActivityParam(name="R", desc="R", fmt="%.1f", min_val=10.0, max_val=30.0, value=20.0),
        }

    def handle_event(self, event_dict):
        super(SpirographActivity, self).handle_event(event_dict)

