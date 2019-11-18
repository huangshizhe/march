#!/usr/bin/env python

import numpy as np
import rospy
import string

current_gains = [1,1,1, 2,2,2, 3,3,3, 4,4,4, 5,5,5, 6,6,6, 7,7,7, 8,8,8]
needed_gains = [2,2,2, 3,3,3, 4,4,4, 5,5,5, 6,6,6, 7,7,7, 8,8,8, 9,9,9]

def interpolate(needed_gains, current_gains):
    for i in range(24):
        if current_gains[i] != needed_gains[i]:
            current_gains[i] = current_gains[i] + 0.5

    return [current_gains]

print(interpolate(current_gains,needed_gains))


