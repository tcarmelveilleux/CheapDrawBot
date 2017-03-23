#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
TkInter GUI for Drawing Robots

Author: tennessee
Created on: 2017-03-15

Copyright 2017, Tennessee Carmel-Veilleux
"""

from __future__ import print_function
import threading
import matplotlib
matplotlib.use('TkAgg')

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
# implement the default mpl key bindings
from matplotlib.backend_bases import key_press_handler
import matplotlib.pyplot as plt
from activities.spirograph import SpirographActivity

from matplotlib.figure import Figure
import sys
if sys.version_info[0] < 3:
    import Tkinter as Tk
    import ttk
else:
    import tkinter as Tk
    from tkinter import ttk

import Queue
import hmi_driver

class RobotController(object):
    def __init__(self, **kwargs):
        self.queue = Queue.Queue()
        self.running = True
        self.thread = threading.Thread(name="RobotController", target=self.process())
        self.data_lock = threading.Lock()

    def handle_controller_event(self, event):
        pass

    def process(self):
        while self.running:
            event = self.queue.get()
            if event is False:
                self.running = False
                continue

            print(event)


#root = Tk.Tk()
#root.wm_title("Embedding in TK")


#f = Figure(figsize=(5, 4), dpi=100)
#a = f.add_subplot(111)
#t = arange(0.0, 3.0, 0.01)
#s = sin(2*pi*t)

#a.plot(t, s)
#
# def _quit():
#     root.quit()     # stops mainloop
#     root.destroy()  # this is necessary on Windows to prevent
#                     # Fatal Python Error: PyEval_RestoreThread: NULL tstate
#
# button = Tk.Button(master=root, text='Quit', command=_quit)
# button.pack(side=Tk.BOTTOM)

#Tk.mainloop()
# If you put root.destroy() here, it will cause an error if
# the window is closed with the window manager.

class RobotControlFrame(object):
    def __init__(self, master, event_handler, activities, **kwargs):
        s = ttk.Style()
        s.configure('.', font=('Helvetica', 12))

        self._frame = ttk.Frame(master)
        self._frame.pack()
        self._event_handler = event_handler

        self._title_label = ttk.Label(self._frame, text="Roboto")
        self._title_label.pack(side="top")

        # Figures for plots
        self._pos_figure = Figure(figsize=(8, 4), dpi=100)
        self._pos_xy_axis = self._pos_figure.add_subplot(121)
        self._pos_polar_axis = self._pos_figure.add_subplot(122)

        # Matplotlib canvas to show plots
        self._canvas = FigureCanvasTkAgg(self._pos_figure, master=self._frame)
        self._canvas.show()
        self._canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        # Matplotlib tools
        self._toolbar_frame = ttk.Frame(master=self._frame)
        self._toolbar = NavigationToolbar2TkAgg(self._canvas, self._toolbar_frame)
        self._toolbar.update()

        self._canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
        self._toolbar_frame.pack(side=Tk.TOP, fill=Tk.BOTH)
        self._canvas.mpl_connect('key_press_event', self.on_key_event)

        # Parameters
        self._params = {}

        # Operations
        self._oper_notebook = ttk.Notebook(master=self._frame)

        for activity in activities:
            activity_frame = activity.make_params_panel(master=self._oper_notebook)
            self._oper_notebook.add(activity_frame, text=activity.name)

        # HPGL
        self._hpgl_param_frame = ttk.Frame(master=self._oper_notebook)
        Tk.Label(self._hpgl_param_frame, text="hpgl").pack(side=Tk.TOP)
        self._hpgl_param_frame.pack(side=Tk.TOP, fill=Tk.X)

        self._oper_notebook.add(self._hpgl_param_frame, text="HPGL")

        self._oper_notebook.pack(side=Tk.TOP, fill=Tk.X)

        # Update button
        self._update_button = Tk.Button(master=self._frame, text="Update", command=lambda: self._event_handler({"event": "update"}))
        self._update_button.pack(side=Tk.BOTTOM, fill=Tk.BOTH)

    def get_param(self, name):
        return self._params[name]

    def get_canvas(self):
        return self._canvas

    def get_pos_figure(self):
        return self._pos_figure

    def get_pos_xy_axis(self):
        return self._pos_xy_axis

    def get_pos_polar_axis(self):
        return self._pos_polar_axis

    def on_key_event(self, event):
        print('you pressed %s' % event.key)
        key_press_handler(event, self._canvas, self._toolbar)

    def get_tk_frame(self):
        return self._frame

class SpirographAppController(object):
    def __init__(self):
        self.params = {"turns": 6, "l": 0.5, "k": 0.25, "R": 25.0}

class DrawBotGuiController(object):
    def __init__(self, app_frame):
        self._app_frame = app_frame
        self._timer_interval_ms = 100

        self._mode = "spirograph"
        self._data_lock = threading.Lock()
        self._params_dirty = False
        self._params = {
            "spirograph": {"turns": 6,
                           "l": 0.5, "l_min": 0.01, "l_max": 0.99,
                           "k": 0.25, "k_min": 0.01, "k_max": 0.99,
                           "R": 25.0, "R_min": 10.0, "R_max": 30.0}
        }

        self._hmi_handlers = {
            "spirograph": self._handle_hmi_event_spirograph
        }
        self._param_handlers = {
            "spirograph": self._update_params_spirograph
        }

        # Bootstrap timer update
        self._app_frame.after(100, self.on_timer)

    def _update_params(self):
        """
        Transfer controller model copy of params to view

        :param params:
        :return:
        """
        handler = self._param_handlers.get(self._mode)
        if handler is not None:
            handler()

    def _update_params_spirograph(self):
        with self._data_lock:
            params = self._params["spirograph"].copy()

        robot_control_frame = self._app_frame.get_robot_control_frame()
        for param_name in ["l", "k", "R"]:
            robot_control_frame.set_spirograph_param(param_name, params[param_name])

        # XXX: Recompute spirograph here
        # XXX: Replot spirograph here

    def handle_hmi_event(self, event):
        handler = self._hmi_handlers.get(self._mode)
        if handler is not None:
            handler(event)

    def _handle_hmi_event_spirograph(self, event):
        mappings = {"axis0": "l", "axis1": "k", "axis2": "R"}

        axis = event["axis"]
        if axis not in mappings:
            return
        param_name = mappings[axis]

        with self._data_lock:
            min_val = self._params["spirograph"]["%s_min" % param_name]
            max_val = self._params["spirograph"]["%s_max" % param_name]
            old_value = self._params["spirograph"][param_name]

            new_value = min_val + ((float(event["value"]) / 1024.0) * (max_val - min_val))

            if new_value != old_value:
                print("Updating spirograph variable %s to %.3f" % (param_name, new_value))
                self._params_dirty = True

            self._params["spirograph"][param_name] = new_value

    def on_timer(self):
        # Update UI if any params are dirty
        dirty = False

        with self._data_lock:
            if self._params_dirty:
                dirty = True
                self._params_dirty = False

        if dirty:
            self._update_params()

        # Schedule next update
        self._app_frame.after(self._timer_interval_ms, self.on_timer)

class AppFrame(object):
    def __init__(self, master=None, **kwargs):
        self.frame = ttk.Frame(master)
        self.frame.pack()

        self.controller = DrawBotGuiController(self)

        activities = []
        spirograph_activity = SpirographActivity(master=self.frame, controller=self.controller)
        activities.append(spirograph_activity)

        self.robot_control_frame = RobotControlFrame(self.frame, self._handle_event, activities)

        # Init HMI driver if requested
        self.hmi_driver = None
        axes_configs = [
            {"name": "axis0", "centered": False},
            {"name": "axis1", "centered": False},
            {"name": "axis2", "centered": False},
            {"name": "joy_x", "centered": True},
            {"name": "joy_y", "centered": True},
            {"name": "button0", "centered": False}]
        if "hmi_port" in kwargs:
            self.hmi_driver = hmi_driver.RobotHMIDriver(kwargs.get("hmi_port"), self.controller.handle_hmi_event, axes_configs)

    def _handle_event(self, event_dict):
        callback_name = "on_%s" % event_dict["event"]
        if hasattr(self, callback_name) and callable(getattr(self, callback_name)):
            getattr(self, callback_name)(event_dict)

    def get_robot_control_frame(self):
        return self.robot_control_frame

    def on_update(self, event_dict):
        event_type = event_dict["event"]
        if event_type == "update":
            turns = float(self.robot_control_frame.get_param("turns")["value"])
            l = self.robot_control_frame.get_param("l")["value"]
            k = self.robot_control_frame.get_param("k")["value"]
            R = self.robot_control_frame.get_param("R")["value"]
            self._update_plot(turns, l, k, R)

    def _update_plot(self, turns, l, k, R):
        n_turns = turns
        quality = 4
        scale = 2 * np.pi / (60.0 * quality)
        max_points = n_turns * 2.0 * np.pi * n_turns / scale

        theta = np.arange(0, max_points) * scale
        print(theta)
        x = R * ((1 - k) * np.cos(theta) + (l * k) * np.cos(((1 - k) / k) * theta))
        y = R * ((1 - k) * np.sin(theta) - (l * k) * np.sin(((1 - k) / k) * theta))

        print("Updating plot")
        canvas = self.robot_control_frame.get_canvas()
        ax = self.robot_control_frame.get_pos_xy_axis()

        ax.clear()
        ax.plot(x, y, 'b-')
        ax.axis([-R, R, -R, R])
        canvas.draw()

    def after(self, num_ms, handler):
        self.frame.after(num_ms, handler)

def center(toplevel):
    # From Wayner Werner's answer at http://stackoverflow.com/a/3353112
    toplevel.update_idletasks()
    w = toplevel.winfo_screenwidth()
    h = toplevel.winfo_screenheight()
    size = tuple(int(_) for _ in toplevel.geometry().split('+')[0].split('x'))
    x = w/2 - size[0]/2
    y = h/2 - size[1]/2
    toplevel.geometry("%dx%d+%d+%d" % (size + (x, y)))

def main():
    root = Tk.Tk()
    app = AppFrame(root) #, hmi_port="COM37")
    center(root)
    root.mainloop()

if __name__ == '__main__':
    main()