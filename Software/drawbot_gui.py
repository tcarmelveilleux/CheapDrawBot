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
import logging
matplotlib.use('TkAgg')

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
# implement the default mpl key bindings
from matplotlib.backend_bases import key_press_handler
import matplotlib.pyplot as plt
from activities.spirograph import SpirographActivity
from drivers.cheapdrawbot import build_cheap_drawbot

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
    def __init__(self, master, drawbot, activities, **kwargs):
        self._activity_updated = False
        self._drawbot = drawbot
        self._timer_interval_ms = 100
        self._logger = logging.getLogger("RobotControlFrame")
        self._activities = activities
        self._current_activity = activities[0]

        self._mode = "spirograph"
        self._data_lock = threading.Lock()

        #######################
        s = ttk.Style()
        s.configure('.', font=('Helvetica', 12))

        self._frame = ttk.Frame(master)
        self._frame.pack()

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
            activity_frame = activity.make_activity_panel(master=self._oper_notebook)
            self._oper_notebook.add(activity_frame, text=activity.name)

        # HPGL
        self._hpgl_param_frame = ttk.Frame(master=self._oper_notebook)
        Tk.Label(self._hpgl_param_frame, text="hpgl").pack(side=Tk.TOP)
        self._hpgl_param_frame.pack(side=Tk.TOP, fill=Tk.X)

        self._oper_notebook.add(self._hpgl_param_frame, text="HPGL")

        self._oper_notebook.pack(side=Tk.TOP, fill=Tk.X)

        # Update button
        self._update_button = Tk.Button(master=self._frame, text="Update", command=lambda: self.handle_event({"event": "update"}))
        self._update_button.pack(side=Tk.BOTTOM, fill=Tk.BOTH)

        ##################

        # Bootstrap timer update
        self._frame.after(100, self.on_timer)

    def handle_event(self, event_dict):
        callback_name = "on_%s" % event_dict["event"]
        if hasattr(self, callback_name) and callable(getattr(self, callback_name)):
            getattr(self, callback_name)(event_dict)
        else:
            self._logger.warn("Could not process event: %s", event_dict)

    def on_activity_updated(self, event_dict):
        with self._data_lock:
            self._activity_updated = True

    def _update_plot(self):
        self._logger.info("Updating plot")
        self._canvas.draw()

    def on_timer(self):
        # Update UI if any params are dirty
        need_update = False

        with self._data_lock:
            if self._activity_updated:
                need_update = True
                self._activity_updated = False

        if need_update:
            self._current_activity.update_geometry()
            ax = self._pos_xy_axis
            ax.clear()
            self._drawbot.kine.draw_robot_preview(ax=ax)
            #self._current_activity.draw_preview(ax=ax)
            self._update_plot()

        # Schedule next update
        self._frame.after(self._timer_interval_ms, self.on_timer)

    def get_param(self, name):
        return self._params[name]

    def get_canvas(self):
        return self._canvas

    def get_pos_figure(self):
        return self._pos_figure

    def get_pos_polar_axis(self):
        return self._pos_polar_axis

    def on_key_event(self, event):
        print('you pressed %s' % event.key)
        key_press_handler(event, self._canvas, self._toolbar)

    def get_tk_frame(self):
        return self._frame

class DrawbotApp(object):
    def __init__(self, master=None, **kwargs):
        self.frame = ttk.Frame(master)
        self.frame.pack()

        # TODO: Support multiple robots!
        drawbot = build_cheap_drawbot()

        activities = []
        spirograph_activity = SpirographActivity(parent=self, drawbot=drawbot)
        activities.append(spirograph_activity)

        self.robot_control_frame = RobotControlFrame(self.frame, drawbot, activities)

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

    def after(self, num_ms, handler):
        self.frame.after(num_ms, handler)

    def handle_event(self, event_dict):
        self.robot_control_frame.handle_event(event_dict)

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

    def handle_hmi_event(self, event):
        handler = self._hmi_handlers.get(self._mode)
        if handler is not None:
            handler(event)

def center(toplevel):
    # From Wayner Werner's answer at http://stackoverflow.com/a/3353112
    toplevel.update_idletasks()
    w = toplevel.winfo_screenwidth()
    h = toplevel.winfo_screenheight()
    size = tuple(int(_) for _ in toplevel.geometry().split('+')[0].split('x'))
    x = w/2 - size[0]/2
    y = h/2 - size[1]/2
    toplevel.geometry("%dx%d+%d+%d" % (size + (x, y)))

def _setup_logging():
    logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(name)-12s %(levelname)-8s: %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

def main():
    _setup_logging()
    root = Tk.Tk()
    app = DrawbotApp(root) #, hmi_port="COM37")
    center(root)
    root.mainloop()

if __name__ == '__main__':
    main()