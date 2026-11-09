"""Expose the pure trajectory checks to colcon's unittest discovery."""

import unittest

from . import test_evaluation, test_trajectory


class FigureEightTests(unittest.TestCase):
    def test_endpoint_and_derivatives(self):
        test_trajectory.test_endpoint_and_derivatives_are_zero()

    def test_analytic_derivatives(self):
        test_trajectory.test_analytic_derivatives_and_peak_speed()

    def test_ramp_boundaries(self):
        test_trajectory.test_ramp_boundaries_are_continuous()

    def test_invalid_parameters(self):
        test_trajectory.test_invalid_parameters_are_rejected()

    def test_percentile_and_acceptance(self):
        test_evaluation.test_percentile_and_acceptance()

    def test_safety_event(self):
        test_evaluation.test_safety_event_fails_acceptance()
