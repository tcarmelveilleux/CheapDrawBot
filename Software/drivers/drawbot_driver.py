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
import json
import numpy as np
from collections import deque, namedtuple

class DrawbotDriverException(IOError):
    pass


class DrawbotCommand(object):
    pass


class DrawbotPenUp(DrawbotCommand):
    def __init__(self, height_mm=10.0):
        self._height_mm = height_mm

    @property
    def height_mm(self):
        return self._height_mm


class DrawbotPenDown(DrawbotCommand):
    def __init__(self, height_mm=0.0):
        self._height_mm = height_mm

    @property
    def height_mm(self):
        return self._height_mm


class DrawbotPenGoto(DrawbotCommand):
    def __init__(self, position, is_native):
        self._position = np.asarray(position, dtype="float64")
        self._is_native = is_native

    @property
    def position(self):
        return self._position

    @property
    def is_native(self):
        return self._is_native


class DrawbotDrawPath(DrawbotCommand):
    def __init__(self, path_points, is_native=False):
        self._path_points = np.asarray(path_points, dtype="float64")
        self._is_native = is_native

    @property
    def path_points(self):
        return self._path_points

    @property
    def is_native(self):
        return self._is_native


class DrawbotAbort(DrawbotCommand):
    def __init__(self):
        pass


class DrawbotDriver(object):
    def __init__(self, drawbot_kinematics, *args, **kwargs):
        self._drawbot_kine = drawbot_kinematics
        self._connected = False
        # Hint for delay between point updates
        self._point_delay_sec = kwargs.get("point_delay_ms", 0.01)
        self._pen_diameter_mm = kwargs.get("pen_diameter_mm", 0.25)
        self._thread = threading.Thread(target=self._process, name=kwargs.get("thread_name", "drawbot_driver"))
        self._thread.daemon = kwargs.get("daemon", True)
        self._running = False
        self._queue = Queue.Queue()
        self._drawing_prog = deque()

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

    @property
    def kine(self):
        return self._drawbot_kine

    def connect(self, port_id, **kwargs):
        if self._connected:
            self._logger.warn("Already connected, ignoring!")
            return

        if not self._thread.is_alive():
            self._running = True
            self.connect_impl(port_id)
            self._thread.start()
            

    def disconnect(self):
        if not self._connected:
            pass

    def get_port_list(self):
        """
        Get all available ports for this robot. Each port is a dict of the format

        {"port_id": port_id_string, "description": description_string}

        A port id is a string that, when given to connect(), would connect to a given robot.
        For a serial-controlled robot, it would be a platform-specific serial device name.
        For a complex USB interface robot, it could be some driver-specific value which the
        driver can recognize and use to connect.

        :return: a list of dicts as described above.
        """
        port_list = self.get_port_list_impl()
        return sorted(port_list, key=lambda x: x.get("port_id"))

    def abort_path(self):
        self._queue.put(DrawbotAbort())

    def pen_up(self, height_mm=10.0):
        self._queue.put(DrawbotPenUp(height_mm=height_mm))

    def pen_down(self, height_mm=0.0):
        self._queue.put(DrawbotPenDown(height_mm=height_mm))

    def goto(self, position, is_native=False):
        self._queue.put(DrawbotPenGoto(position, is_native=is_native))

    def draw_path(self, path_points, is_native=False):
        self._queue.put(DrawbotDrawPath(path_points, is_native=is_native))

    def shutdown(self, timeout=1.0):
        if self._thread.is_alive():
            self._queue.put(False)
            self._thread.join(timeout)
            if self._thread.is_alive():
                self._logger.error("Could not join robot driver thread to shutdown!")

    def _process(self):
        self._logger.info("Drawbot driver thread started")
        while self._running:
            cmd = None
            try:
                cmd = self._queue.get(block=True,timeout=self._point_delay_sec)
            except Queue.Empty:
                # No new command, run next robot command
                if len(self._drawing_prog) == 0:
                    continue

                drawing_cmd = self._drawing_prog.popleft()
                self._execute(drawing_cmd)

            # If no command to execute, go back to waiting
            if cmd is None:
                continue

            if cmd is False:
                # Shutdown requested
                self._running = False
                continue

            # Handle all commands
            if isinstance(cmd, DrawbotDrawPath):
                path = cmd.path_points
                is_native = cmd.is_native

                # Convert to natives
                if not is_native:
                    points, path = self._drawbot_kine.gen_path(path, self._pen_diameter_mm / 2.0)
                    is_native = True

                for idx in xrange(path.shape[0]):
                    self._drawing_prog.append({"cmd": ("goto_native" if is_native else "goto_point"), "point": path[idx,:]})
            elif isinstance(cmd, DrawbotPenUp):
                self._drawing_prog.append({"cmd": "pen_up", "height_mm": cmd.height_mm})
            elif isinstance(cmd, DrawbotPenDown):
                self._drawing_prog.append({"cmd": "pen_down", "height_mm": cmd.height_mm})
            elif isinstance(cmd, DrawbotPenGoto):
                is_native = cmd.is_native

                # Convert to natives
                if not is_native:
                    try:
                        # Find IK
                        natives = self._drawbot_kine.inverse_kine(cmd.position)
                        is_native = True
                    except:
                        self._logger.exception("Could not go ot native!")
                        continue
                else:
                    natives = cmd.position

                self._drawing_prog.append(
                    {"cmd": ("goto_native" if is_native else "goto_point"), "point": natives})
            elif isinstance(cmd, DrawbotAbort):
                self._drawing_prog.append({"cmd": "abort"})

        self._logger.info("Drawbot driver thread exited")
        return

    def _execute(self, drawing_cmd):
        if drawing_cmd["cmd"] == "goto_point":
            self._logger.info("goto_point: %s", drawing_cmd["point"])
        elif drawing_cmd["cmd"] == "goto_native":
            #sself._logger.info("goto_native: %s", drawing_cmd["point"])
            self.set_natives_impl(drawing_cmd["point"])
        elif drawing_cmd["cmd"] in ("pen_up", "pen_down"):
            self.set_pen_height_impl(height_mm=drawing_cmd["height_mm"])
        else:
            self._logger.warn("Unknown command: %s", drawing_cmd)

    def get_port_list_impl(self):
        raise NotImplementedError()

    def connect_impl(self, port_id):
        raise NotImplementedError()

    def disconnect_impl(self):
        raise NotImplementedError()

    def set_pen_height_impl(self, height_mm):
        raise NotImplementedError()

    def set_natives_impl(self, natives):
        raise NotImplementedError()


class DrawbotKinematics(object):
    """

    """
    def __init__(self, *args, **kwargs):
        self.work_area_config = {}
        pass

    def get_kine_hash(self):
        """
        Return a driver-specific hash given the configuration of the robot, which can be used to generate
        a cache key.
        :return: A driver-specific hash string or None if no hash can be computed
        """
        return None

    def load_work_area_config(self):
        """
        Try to load work area config from cached file

        :return: True if work area config was loaded, False otherwise
        """
        hash = self.get_kine_hash()
        if hash is None:
            return False

        try:
            with open("%s_%s_cache.json" % (self.__class__.__name__, hash), "rb") as cache_file:
                work_area_config = json.load(cache_file)
                self.work_area_config.update(work_area_config)
                return True
        except IOError:
            return False

    def save_work_area_config(self):
        """
        Try to save work area config into cache file
        """
        hash = self.get_kine_hash()
        with open("%s_%s_cache.json" % (self.__class__.__name__, hash), "wb+") as cache_file:
            json.dump(self.work_area_config, cache_file, indent=2)

    def respects_external_constraints(self, end_point, thetas):
        return False

    def forward_kine(self, thetas, **kwargs):
        # Position of end-effector
        end_points = np.zeros(thetas.shape)

        # Other columns (e.g. phi1, phi2, cx, cy), stacked at the end of the end_points. Could be as many columns as you want
        internal_stuff = np.zeros((thetas.shape[0], 4))

        return np.column_stack((end_points, internal_stuff))

    def inverse_kine(self, end_points, **kwargs):
        # end_points: array of points of end effector (x, y) for which to find the natives
        natives = np.zeros(end_points.shape)

        # Other columns (e.g. phi1, phi2, cx, cy), stacked at the end of the end_points. Could be as many columns as you want
        internal_stuff = np.zeros((end_points.shape[0], 4))

        if self.respects_external_constraints(end_points, natives):
            return np.column_stack((natives, internal_stuff))
        else:
            raise ValueError("No reverse kine solution found")

    # Return an array of only the points that fit the IK of the robot
    def trim_unfeasible(self, points):
        feasible_points = []
        for point in points:
            try:
                self.inverse_kine(point)
                feasible_points.append(points)
            except ValueError:
                feasible_points.append(point)

        return np.asarray(feasible_points)

    def get_work_area(self, spatial_res_mm=0.5, theta_res_rad=0.05, **kwargs):
        # Samples of the work area
        work_points = []
        # List of thetas scanned to have been feasible
        work_natives = []
        # Closed path around work area
        work_path = []
        return work_points, work_natives, work_path

    def get_work_area_centroid(self):
        return np.asarray(self.work_area_config.get("work_centroid", [0.0, 0.0]))

    def draw_robot_preview(self, ax, show_robot=False, show_work_area=True, **kwargs):
        """
        Draw preview of robot and work area on matplotlib axis
        :param ax:
        :param show_robot:
        :param show_work_area:
        """
        pass
