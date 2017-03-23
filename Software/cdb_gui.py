#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
TkInter GUI for CheapDrawBot

Author: tennessee
Created on: 2017-03-15

Copyright 2017, Tennessee Carmel-Veilleux

"""

import Tkinter

class RobotControlFrame(object):
    def __init__(self, master=None, **kwargs):
        self.frame = Tkinter.Frame(master)
        self.frame.pack()

        self._title_label = Tkinter.Label(self.frame, text="Roboto")
        self._title_label.pack(side="top")

    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class AppFrame(object):
    def __init__(self, master=None, **kwargs):
        self.frame = Tkinter.Frame(master)
        self.frame.pack()

        self.RobotControlFrame = RobotControlFrame(self.frame)

def main():
    root = Tkinter.Tk()
    app = AppFrame(root)
    root.mainloop()

if __name__ == '__main__':
    main()