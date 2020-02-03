#!/usr/bin/env python
import rosunit

from .one_step_linear_interpolation_test import OneStepLinearInterpolationTest

PKG = 'march_gain_scheduling'

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_linear_interpolation', OneStepLinearInterpolationTest)
