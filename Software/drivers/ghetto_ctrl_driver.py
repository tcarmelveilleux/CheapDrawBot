#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Arduino-based really ghetto robot robot human-machine-interface driver

Author: tennessee
Created on: 2017-03-17

Copyright 2017, Tennessee Carmel-Veilleux. All rights reserved.
"""

from __future__ import print_function
import sys
import time
import serial
import threading

class GhettoCtrlDriver(object):
    """
    Super-el-cheapo robot HMI controller
    """
    def __init__(self, port, event_handler, axes_configs):
        """
        Start a robot HMI driver.

        Axes are configured with following dict:
        { "name": "name_of_axis_here_used_in_events",
          "centered": True|False, # If true, a "center" value will be calculated and removed from each reading}

        :param port: Serial port to use
        :param event_handler: Event handler that receives the events
        :param axes_configs: List of axis configuration dicts. See above
        """
        self.port = port
        self.event_handler = event_handler
        self.num_axes = len(axes_configs)
        self.axes_configs = axes_configs
        self.centers = [0] * self.num_axes
        self.last_values = [-1] * self.num_axes

        self.priming = True
        self.priming_idx = 0
        self.num_priming_samples = 16

        self.serial = serial.Serial(port=port, baudrate=115200, timeout=0.2)
        self.running = True

        self.thread = threading.Thread(target=self.process, name="hmi_driver.%s" % port)
        self.thread.daemon = True
        self.thread.start()

    def process(self):
        while self.running:
            line = self.serial.readline().strip()
            if len(line) == 0:
                continue

            # Format of line is "CTRL,0,1,2,3,4,5,6,7,...\n", where 0..n are integer values for axes
            if not line.startswith("CTRL,"):
                continue

            tokens = line.split(",")
            if len(tokens) != self.num_axes + 1:
                continue

            # Have enough tokens for event: generate it
            axis_values = [int(tokens[1 + axis]) for axis in range(self.num_axes)]
            ts = time.time()

            # Handle priming centers at startup
            if self.priming:
                self.priming_idx += 1
                if self.priming_idx == self.num_priming_samples:
                    # Done priming, stop process and average
                    self.priming = False
                    for axis_idx in range(self.num_axes):
                        self.centers[axis_idx] /= self.num_priming_samples
                else:
                    # Not done priming, accumulate
                    for axis_idx in range(self.num_axes):
                        if self.axes_configs[axis_idx]["centered"]:
                            self.centers[axis_idx] += axis_values[axis_idx]
            else:
                for axis_idx in range(self.num_axes):
                    value = axis_values[axis_idx]
                    if value != self.last_values[axis_idx]:
                        self.last_values[axis_idx] = value
                        if self.axes_configs[axis_idx]["centered"]:
                            event_value = (value - self.centers[axis_idx]) / 512.0
                            event_value = max(-1.0, min(1.0, event_value))
                        else:
                            event_value = value / 1023.0

                        event = {"ts": ts, "axis": self.axes_configs[axis_idx]["name"], "value": event_value}
                        if self.event_handler is not None:
                            self.event_handler(event)

    def shutdown(self):
        self.running = False
        self.thread.join(1.0)

if __name__ == '__main__':
    def event_handler(event):
        print(event)

    driver = None
    try:
        serial_port = sys.argv[1]
        axes_configs = [
            {"name": "axis0", "centered": False},
            {"name": "axis1", "centered": False},
            {"name": "axis2", "centered": False},
            {"name": "joy_x", "centered": True},
            {"name": "joy_y", "centered": True},
            {"name": "button0", "centered": False}]

        driver = GhettoCtrlDriver(serial_port, event_handler, axes_configs)
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        if driver is not None:
            driver.shutdown()