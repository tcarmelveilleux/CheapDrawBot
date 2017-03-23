#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Robot Controller API

Author: tennessee
Created on: 2017-03-21

Copyright 2017, Tennessee Carmel-Veilleux.
"""

class RobotController(object):
    def __init__(self, drawbot_driver):
        self._drawbot_driver = drawbot_driver
