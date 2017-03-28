#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Common hash functions for cache hashes

Author: tennessee
Created on: 2017-03-27

Copyright 2017, Tennessee Carmel-Veilleux. All rights reserved
"""
from __future__ import print_function
import hashlib
import struct
import base64

def params_hash(iterable_of_numeric):
    """
    Return a hash made of the 16 first base64 chars of the MD5 hash of all numbers in the iterable_of_numeric
    as floats. Casts all numbers to Python floats and serializes to binary, then MD5 hashes and truncates the
    base64 of the hash. Generates a generally-random-enough but deterministic 16 chars hash from a list
    of numerical parameters.

    :param iterable_of_numeric: Iterable sequence of numerical types (castable to float())
    :return: A 16-long string of a hash of the given numbers
    """
    parts = [0.0]
    for element in iterable_of_numeric:
        parts.append(float(element))
    packed = struct.Struct("d" * len(parts)).pack(*tuple(parts))
    md5 = hashlib.md5()
    md5.update(packed)
    return base64.b64encode(md5.digest())[:16]

if __name__ == '__main__':
    print("Test 1: %s", params_hash([1.0, 5, 12, -23e6]))
    print("Test 2: %s should equal Test 1", params_hash([1.0, 5, 12, -23e6]))
    print("Test 3: %s", params_hash([]))
    print("Test 4: %s", params_hash([0]))
    print("Test 5: %s should equal Test 5", params_hash([0.0]))
