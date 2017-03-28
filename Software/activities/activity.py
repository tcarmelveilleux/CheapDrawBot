#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Base class for activities. Activities have a parameters panel, handle events and
can make requests to the robot controller.

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""
import logging
import sys
if sys.version_info[0] < 3:
    import Tkinter as Tk
    import ttk
else:
    import tkinter as Tk
    from tkinter import ttk

class ActivityParam(object):
    def __init__(self, name, desc, min_val, max_val, value, fmt):
        self._name = name
        self._desc = desc
        self._min_val = min_val
        self._max_val = max_val
        self._value = value
        self._fmt = fmt

    @property
    def name(self):
        return self._name

    @property
    def desc(self):
        return self._desc

    @property
    def min_val(self):
        return self._min_val

    @min_val.setter
    def min_val(self, value):
        self._min_val = value

    @property
    def max_val(self):
        return self._max_val

    @max_val.setter
    def max_val(self, value):
        self._max_val = value

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, the_value):
        if the_value > self._max_val:
            self._value = self._max_val
        elif the_value < self._min_val:
            self._value = self._min_val
        else:
            self._value = the_value

    def __str__(self):
        return self._fmt % self._value


class Activity(object):
    def __init__(self, parent, drawbot, *args, **kwargs):
        self._parent = parent
        self._param_frame = None
        self._params = {}
        self._param_ctrls = {}
        self._drawbot = drawbot
        self._drawbot_sequence = []
        self._name = "Unknown"
        self._logger = logging.getLogger("activity.%s" % self.__class__.__name__)

    @property
    def name(self):
        return self._name

    def get_drawbot_sequence(self):
        return self._drawbot_sequence
        #pass
        #self._controller.update_path(self._drawbot_sequence)

    def handle_event(self, event_dict):
        """
        Handle an event coming from the controller or widgets/params

        :param event_dict: Dict containing event data. Only mandatory key is event name: {"event": "event_name_here", ... }
        :return: True if event was handled, false otherwise (good to determine if parent processed while in a child event handler)
        """
        return False

    def num_var_changed(self, num_var, str_var, param):
        # Track string value to slider
        value = num_var.get()
        param.value = value
        str_var.set(str(param))
        # TODO BETTER NOTIFY!
        self.handle_event({"event": "param_changed", "param": param})

    def str_var_changed(self, num_var, param, str_value):
        # Track slider value to string
        try:
            if isinstance(num_var, Tk.DoubleVar):
                value = float(str_value)
            else:
                value = int(str_value, 0)

            num_var.set(value)
            param.value = value
        except ValueError as e:
            # Change nothing on error
            pass

        return True

    def get_param(self, name):
        return self._params[name]

    def _make_entry(self, parent, caption, width=None, **options):
        ttk.Label(parent, text=caption).pack(side=Tk.LEFT)
        entry = ttk.Entry(parent, **options)
        if width:
            entry.config(width=width)
        entry.pack(side=Tk.LEFT)
        return entry

    def _gen_param_ctrl(self, master, param):
        param_frame = ttk.Frame(master=master)
        str_var = Tk.StringVar(master=param_frame, value=str(param))
        if isinstance(param.value, int):
            # Integers
            num_var = Tk.IntVar(master=param_frame, value=param.value)
        else:
            # Reals
            num_var = Tk.DoubleVar(master=param_frame, value=param.value)
        num_var.trace_variable("w", lambda *args: self.num_var_changed(num_var, str_var, param))

        param_entry = self._make_entry(param_frame, "%s:" % param.desc, textvariable=str_var,
                                        validate="focusout",
                                        validatecommand=lambda: self.str_var_changed(num_var, param, str_var.get()))

        param_entry.pack(side=Tk.LEFT, fill=Tk.Y, padx=5)
        param_scale = ttk.Scale(param_frame, from_=param.min_val, to=param.max_val, orient=Tk.HORIZONTAL, variable=num_var)
        param_scale.pack(side=Tk.RIGHT, fill=Tk.X, expand=1, padx=5)
        param_frame.pack(side=Tk.TOP, fill=Tk.X, expand=1, pady=5)
        return param_frame

    def make_activity_panel(self, master):
        """
        Generate frame containing all controls for activity. Default panel includes slider controls for
        all parameters.

        :param master: master for the frame
        :return: the frame created (e.g. for Notebook insertion)
        """
        self._param_frame = ttk.Frame(master=master)
        for name, param in self._params.items():
            self._param_ctrls[name] = self._gen_param_ctrl(self._param_frame, param)
        self._param_frame.pack(side=Tk.TOP, fill=Tk.X)
        return self._param_frame

    def draw_preview(self, ax):
        """
        Draw a preview of the robot path on the given Matplotlib axis
        :param ax: matplotlib axis on which to draw. Externally provided
        """
        raise NotImplementedError()

    def update_geometry(self):
        # Recompute the internal geometry on request from the controller.
        raise NotImplementedError()