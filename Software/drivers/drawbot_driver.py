#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Base class for all drawbot drivers

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""
import threading
import Queue
import logging
from collections import deque

class DrawbotDriverException(IOError):
    pass

class DrawbotDriver(object):
    def __init__(self, drawbot_kinematics, *args, **kwargs):
        self._drawbot_kine = drawbot_kinematics
        self._connected = False
        # Hint for delay between point updates
        self._point_delay_sec = kwargs.get("point_delay_ms", 0.0)
        self._pen_diameter_mm = kwargs.get("pen_diameter_mm", 0.5)
        self._thread = threading.Thread(target=self._process, name=kwargs.get("thread_name", "drawbot_driver"))
        self._thread.daemon = kwargs.get("daemon", True)
        self._running = False
        self._queue = Queue.Queue()
        self._thetas_path = deque()

        self._logger = logging.getLogger("drawbot_driver")

    @property
    def connected(self):
        return self._connected

    @property
    def point_delay_sec(self):
        return self._point_delay_sec

    @point_delay_sec.setter
    def point_delay_sec(self, value):
        self._queue.put({"cmd": "set_point_delay_sec", "value": value})

    @property
    def pen_diameter_mm(self):
        return self._pen_diameter_mm

    def connect(self, **kwargs):
        if self._connected:
            self._logger.warn("Already connected, ignoring!")
            return

        if not self._thread.is_alive():
            self._thread.start()

    def disconnect(self):
        if not self._connected:
            pass

    def abort_path(self):
        self._queue.put({"cmd": "abort"})

    def pen_up(self, height_mm=10.0):
        self._queue.put({"cmd": "set_pen_height", "height_mm": height_mm})

    def pen_down(self, height_mm=0.0):
        self._queue.put({"cmd": "set_pen_height", "height_mm": height_mm})

    def goto(self, end_point):
        self._queue.put({"cmd": "goto", "end_point": end_point})

    def goto_thetas(self, thetas):
        self._queue.put({"cmd": "goto_thetas", "thetas": thetas})

    def draw_path(self, path_points):
        self._queue.put({"cmd": "draw_path", "path_points": path_points})

    def draw_thetas_path(self, path_thetas):
        self._queue.put({"cmd": "draw_thetas_path", "path_thetas": path_thetas})

    def shutdown(self, timeout=1.0):
        if self._thread.is_alive():
            self._queue.put({"cmd": "shutdown"})
            self._thread.join(timeout)
            if self._thread.is_alive():
                self._logger.error("Could not join robot driver thread to shutdown!")

    def _process(self):
        # TODO: impl!
        while self._running:
            cmd = self._queue.get(block=True)
            if cmd["cmd"] == "shutdown":
                self._running = False

        return

    def connect_impl(self):
        raise NotImplementedError()

    def disconnect_impl(self):
        raise NotImplementedError()

    def set_pen_height_impl(self):
        raise NotImplementedError()

    def set_thetas_impl(self, thetas):
        raise NotImplementedError()

class DrawbotKinematics(object):
    """

    """
    def __init__(self, *args, **kwargs):
        pass

    def respects_external_constraints(self, end_point, thetas):
        return False

    def forward_kine(self, thetas, **kwargs):
        # Position of end-effector
        end_point = (0.0, 0.0)

        # dict of random bits to return from forward kinematics geometry
        internal_stuff = { "phi1": 0.0, "phi2": 0.0, "cx": 0.0, "cy": 0.0}

        return end_point, internal_stuff

    def inverse_kine(self, end_point, **kwargs):
        # end_point: Point of end effector (x, y) for which to find the theta1/theta2

        thetas = (0.0, 0.0)

        # dict of random bits to return from forward kinematics geometry
        internal_stuff = {"phi1": 0.0, "phi2": 0.0, "cx": 0.0, "cy": 0.0}

        if self.respects_external_constraints(end_point, thetas):
            return thetas, internal_stuff
        else:
            raise ValueError("No reverse kine solution found")

    def get_feasible_area(self, spatial_res_mm=0.5, theta_res_rad=0.05, **kwargs):
        # List of end_points forming a closed path around the feasible area
        feas_path_points = []
        # List of thetas scanned to have been feasible
        feas_thetas = []
        return feas_path_points, feas_thetas
