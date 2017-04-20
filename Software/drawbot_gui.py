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
from activities.draw_hpgl import DrawHpglActivity
from drivers.cheapdrawbot import build_cheap_drawbot
from version import VERSION

from matplotlib.figure import Figure
import sys
if sys.version_info[0] < 3:
    import Tkinter as Tk
    import ttk
else:
    import tkinter as Tk
    from tkinter import ttk

import tkMessageBox
import Queue
#import hmi_driver

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


class RobotDriverFrame(object):
    def __init__(self, parent, master, drawbot, **kwargs):
        self._parent = parent
        self._drawbot = drawbot

        self._frame = ttk.LabelFrame(master=master, text="Drawbot", padding=5)

        # TODO: Add type

        self._port_frame = ttk.Frame(master=self._frame)
        self._port_label = ttk.Label(master=self._port_frame, text="Port:")
        self._port_label.pack(side=Tk.LEFT)

        self._ports = drawbot.get_port_list()
        self._values = ["%(port_id)s, %(description)s" % port for port in self._ports]
        self._port_entry = ttk.Combobox(master=self._port_frame, values=self._values)
        self._port_entry.state(["readonly"])
        self._port_entry.current(0)

        self._port_entry.pack(side=Tk.LEFT)
        self._port_frame.pack(side=Tk.TOP, fill=Tk.X, expand=1)

        # Connection button
        self._connect_button = ttk.Button(master=self._frame, text="Connect",
                                          command=self._on_connect)
        self._connect_button.pack(side=Tk.TOP, fill=Tk.X, expand=1)

        self._disconnect_button = ttk.Button(master=self._frame, text="Disconnect",
                                          command=lambda: self._parent.handle_event({"event": "disconnect"}))
        self._disconnect_button.state(["disabled"])
        self._disconnect_button.pack(side=Tk.TOP, fill=Tk.X, expand=1)

        spacer1 = ttk.Frame(master=self._frame)
        spacer1.pack(side=Tk.TOP, pady=10)

        # Draw button
        s = ttk.Style()
        s.configure('prop.TLabel', font=('Courier', 12))

        self._start_button = ttk.Button(master=self._frame, text="Draw!",
                                      command=lambda: self._parent.handle_event({"event": "drawbot_start"}))
        self._start_button.pack(side=Tk.TOP, fill=Tk.X, expand=1)

        self._stop_button = ttk.Button(master=self._frame, text="Abort!",
                                       command=lambda: self._parent.handle_event({"event": "drawbot_stop"}))
        self._stop_button.state(["disabled"])
        self._stop_button.pack(side=Tk.TOP, fill=Tk.X, expand=1)

        spacer2 = ttk.Frame(master=self._frame)
        spacer2.pack(side=Tk.TOP, pady=10)

        # Location of robot
        self._location_frame = ttk.Frame(master=self._frame)

        ttk.Label(master=self._location_frame, text="X: ").grid(row=0, sticky=Tk.W)
        ttk.Label(master=self._location_frame, text="Y: ").grid(row=1, sticky=Tk.W)
        ttk.Label(master=self._location_frame, text="Z: ").grid(row=2, sticky=Tk.W)

        self._x_var = Tk.StringVar("")
        self._x_label = ttk.Label(master=self._location_frame, textvariable=self._x_var, width=-10, style="prop.TLabel")
        self._x_label.grid(row=0, column=1, sticky=Tk.W)

        self._y_var = Tk.StringVar("")
        self._y_label = ttk.Label(master=self._location_frame, textvariable=self._y_var, width=-10, style="prop.TLabel")
        self._y_label.grid(row=1, column=1, sticky=Tk.W)

        self._z_var = Tk.StringVar("")
        self._z_label = ttk.Label(master=self._location_frame, textvariable=self._z_var, width=-10, style="prop.TLabel")
        self._z_label.grid(row=2, column=1, sticky=Tk.W)

        self._location_frame.pack(side=Tk.TOP, fill=Tk.X, expand=1)

    def _on_connect(self):
        if len(self._ports) == 0:
            return

        port_idx = self._port_entry.current()
        port_id = self._ports[port_idx]["port_id"]

        self._parent.handle_event({"event": "connect", "port_id": port_id})

    def _set_enable(self, control, enabled):
        if enabled:
            control.state(["!disabled"])
        else:
            control.state(["disabled"])

    def set_connect_enable(self, enabled):
        self._set_enable(self._connect_button, enabled=enabled)

    def set_disconnect_enable(self, enabled):
        self._set_enable(self._disconnect_button, enabled=enabled)

    def set_start_enable(self, enabled):
        self._set_enable(self._start_button, enabled=enabled)

    def set_stop_enable(self, enabled):
        self._set_enable(self._stop_button, enabled=enabled)

    def set_location(self, x, y, z, x_fmt="%7.1f", y_fmt="%7.1f", z_fmt="%7.1f"):
        self._x_var.set(x_fmt % x)
        self._y_var.set(y_fmt % y)
        self._z_var.set(z_fmt % z)

    def pack(self, *args, **kwargs):
        self._frame.pack(*args, **kwargs)


class RobotControlFrame(object):
    def __init__(self, master, drawbot, activities, **kwargs):
        self._drawbot = drawbot
        self._timer_interval_ms = 100
        self._logger = logging.getLogger("RobotControlFrame")
        self._activities = activities
        self._current_activity = activities[0]

        self._mode = "spirograph"
        self._data_lock = threading.Lock()

        self._activity_updated = True

        # Robot endpoint position
        self._pos_updated = False
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0

        #######################
        s = ttk.Style()
        s.configure('.', font=('Helvetica', 12))

        self._frame = ttk.Frame(master)
        self._frame.pack()

        self._top_frame = ttk.Frame(master=self._frame)

        self._fig_frame = ttk.Frame(master=self._top_frame)

        # Figures for plots
        self._pos_figure = Figure(figsize=(8, 3), dpi=100)
        self._pos_xy_axis = self._pos_figure.add_subplot(121)
        self._pos_polar_axis = self._pos_figure.add_subplot(122)

        # Matplotlib canvas to show plots
        self._canvas = FigureCanvasTkAgg(self._pos_figure, master=self._fig_frame)
        self._canvas.show()
        self._canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        # Matplotlib tools
        self._toolbar_frame = ttk.Frame(master=self._fig_frame)
        self._toolbar = NavigationToolbar2TkAgg(self._canvas, self._toolbar_frame)
        self._toolbar.update()

        self._canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
        self._toolbar_frame.pack(side=Tk.TOP, fill=Tk.BOTH)
        self._canvas.mpl_connect('key_press_event', self.on_key_event)

        self._fig_frame.pack(side=Tk.LEFT, fill=Tk.BOTH, expand=1)

        # Robot driver frame
        self._robot_driver_frame = RobotDriverFrame(parent=self, master=self._top_frame, drawbot=drawbot)
        self._robot_driver_frame.pack(side=Tk.LEFT, padx=5, ipadx=5, ipady=5)

        self._top_frame.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        # Parameters
        self._params = {}

        # Operations
        self._oper_notebook = ttk.Notebook(master=self._frame)

        for activity in activities:
            activity_frame = activity.make_activity_panel(master=self._oper_notebook)
            activity_frame.pack(side=Tk.TOP, fill=Tk.BOTH, expand=0)
            self._oper_notebook.add(activity_frame, text=activity.name)

        # # HPGL
        # self._hpgl_param_frame = ttk.Frame(master=self._oper_notebook)
        # Tk.Label(self._hpgl_param_frame, text="hpgl").pack(side=Tk.TOP)
        # self._hpgl_param_frame.pack(side=Tk.TOP, fill=Tk.X)
        #
        # self._oper_notebook.add(self._hpgl_param_frame, text="HPGL")

        self._oper_notebook.pack(side=Tk.BOTTOM, fill=Tk.BOTH, expand=0)
        self._oper_notebook.bind("<<NotebookTabChanged>>", self.on_activity_changed)

        ##################

        self._frame.pack(side=Tk.TOP, fill=Tk.BOTH, expand=2)
        # Bootstrap timer update
        self._frame.after(100, self.on_timer)

    def handle_event(self, event_dict):
        callback_name = "on_%s" % event_dict["event"]
        if hasattr(self, callback_name) and callable(getattr(self, callback_name)):
            getattr(self, callback_name)(event_dict)
        else:
            self._logger.warn("Could not process event: %s", event_dict)

    def on_activity_changed(self, event):
        notebook = self._oper_notebook
        activity_idx = notebook.index(notebook.select())
        self._logger.info("%s", repr(activity_idx))
        self._current_activity = self._activities[activity_idx]
        self._logger.info("Selected activity %s", self._current_activity.name)

        # Force refresh of activity
        with self._data_lock:
            self._activity_updated = True

    def on_activity_updated(self, event_dict):
        with self._data_lock:
            self._activity_updated = True

    def on_drawbot_start(self, event_dict):
        self._current_activity.start_drawing()
        self._robot_driver_frame.set_start_enable(False)
        self._robot_driver_frame.set_stop_enable(True)

    def on_drawbot_stop(self, event_dict):
        self._current_activity.stop_drawing()
        self._robot_driver_frame.set_start_enable(True)
        self._robot_driver_frame.set_stop_enable(False)

    # Called by drawbot event loop in DrawbotDriver thread context because we are observer.
    # MAKE SURE TO KEEP SHORT-RUNNING
    def on_drawbot_event(self, drawbot_event):
        if drawbot_event["event"] == "drawbot_position":
            with self._data_lock:
                self._pos_updated = True
                self._x = drawbot_event["x"]
                self._y = drawbot_event["y"]
                self._z = drawbot_event["z"]

    def on_connect(self, event_dict):
        port_id = event_dict["port_id"]
        self._logger.info("Trying to connect to %s", port_id)
        try:
            self._drawbot.connect(port_id=port_id)
        except BaseException as e:
            message = "Could not connect to port '%s'\nException: %s, %s" % (port_id, e.__class__.__name__, str(e))
            title = "Connection Error..."
            tkMessageBox.showerror(title, message)
            return

        self._drawbot.add_event_handler(self.on_drawbot_event)
        self._robot_driver_frame.set_connect_enable(False)
        self._robot_driver_frame.set_disconnect_enable(True)

    def on_disconnect(self, event_dict):
        try:
            self._drawbot.disconnect()
        except BaseException as e:
            pass

        self._drawbot.remove_event_handler(self.on_drawbot_event)
        self._robot_driver_frame.set_connect_enable(True)
        self._robot_driver_frame.set_disconnect_enable(False)

    def _update_plot(self):
        self._logger.info("Updating plot")
        self._canvas.draw()

    def on_timer(self):
        # Update UI if any params are dirty
        need_plots_update = False
        need_pos_update = False

        with self._data_lock:
            if self._activity_updated:
                need_plots_update = True
                self._activity_updated = False

            if self._pos_updated:
                need_pos_update = True
                self._pos_updated = False

        # Update plots
        if need_plots_update:
            self._logger.info("Activity needs update")
            self._current_activity.update_geometry()
            ax = self._pos_xy_axis
            ax.clear()
            ax.axis("equal")
            self._current_activity.draw_preview(ax=ax)
            self._drawbot.kine.draw_robot_preview(ax=ax)
            self._update_plot()

        # Update position
        if need_pos_update:
            self._robot_driver_frame.set_location(self._x, self._y, self._z)

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

        # TODO: Support multiple robots!
        drawbot = build_cheap_drawbot()

        activities = []
        spirograph_activity = SpirographActivity(parent=self, drawbot=drawbot)
        activities.append(spirograph_activity)
        hpgl_activity = DrawHpglActivity(parent=self, drawbot=drawbot)
        activities.append(hpgl_activity)

        self.robot_control_frame = RobotControlFrame(self.frame, drawbot, activities)
        self.frame.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

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
    y = h/2 - size[1]/2 - 30
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
    root.title("Drawbot GUI v%s" % VERSION)
    root.mainloop()

if __name__ == '__main__':
    main()