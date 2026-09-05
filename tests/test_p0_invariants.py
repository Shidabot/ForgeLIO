#!/usr/bin/env python3
"""Dependency-free regression checks for the P0 estimator fixes.

Run with:
    python3 tests/test_p0_invariants.py

These checks complement, but do not replace, a catkin build and rosbag test.
"""

import math
import pathlib
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]


def interpolate(before, after, timestamp):
    """Reference scalar interpolation used to exercise boundary semantics."""
    t0, value0 = before
    t1, value1 = after
    if not t0 <= timestamp <= t1 or not t1 > t0:
        raise ValueError("timestamp is not bracketed")
    alpha = (timestamp - t0) / (t1 - t0)
    return timestamp, (1.0 - alpha) * value0 + alpha * value1


def extract_interval(samples, start, end):
    """Small reference model of PoseEstimation's ordered IMU extraction."""
    if not start < end:
        raise ValueError("invalid interval")
    if len(samples) < 2 or samples[0][0] > start or samples[-1][0] < end:
        raise ValueError("interval is not covered")

    def sample_at(timestamp):
        for index, sample in enumerate(samples):
            if math.isclose(sample[0], timestamp, abs_tol=1e-12):
                return sample
            if sample[0] > timestamp:
                return interpolate(samples[index - 1], sample, timestamp)
        raise ValueError("timestamp is not bracketed")

    result = [sample_at(start)]
    result.extend(sample for sample in samples if start < sample[0] < end)
    result.append(sample_at(end))
    return result


class P0InvariantTests(unittest.TestCase):
    def test_imu_interval_has_exact_interpolated_boundaries(self):
        samples = [(0.98, 0.0), (1.03, 5.0), (1.08, 10.0), (1.13, 15.0)]
        interval = extract_interval(samples, 1.0, 1.1)
        self.assertEqual([sample[0] for sample in interval], [1.0, 1.03, 1.08, 1.1])
        self.assertAlmostEqual(interval[0][1], 2.0)
        self.assertAlmostEqual(interval[-1][1], 12.0)

    def test_adjacent_imu_intervals_share_boundary_without_a_gap(self):
        samples = [(0.98, 0.0), (1.03, 5.0), (1.08, 10.0), (1.13, 15.0)]
        left = extract_interval(samples, 1.0, 1.05)
        right = extract_interval(samples, 1.05, 1.1)
        self.assertEqual(left[-1], right[0])
        self.assertAlmostEqual(left[-1][1], 7.0)

    def test_marginalization_uses_optimizer_coordinates(self):
        source = (ROOT / "include/utils/ceresfunc.h").read_text(encoding="utf-8")
        self.assertIn("dx.segment(idx, size) = x - x0;", source)
        self.assertNotIn("exp(x.segment<3>(3)).inverse()", source)

        # For r = r0 + J(x-x0), the finite-difference derivative is exactly J.
        jacobian = [2.0, -3.0, 0.5]
        x0 = [0.1, -0.2, 0.3]
        epsilon = 1e-7
        for column in range(3):
            plus = list(x0)
            minus = list(x0)
            plus[column] += epsilon
            minus[column] -= epsilon
            r_plus = sum(j * (x - x_ref) for j, x, x_ref in zip(jacobian, plus, x0))
            r_minus = sum(j * (x - x_ref) for j, x, x_ref in zip(jacobian, minus, x0))
            derivative = (r_plus - r_minus) / (2.0 * epsilon)
            self.assertAlmostEqual(derivative, jacobian[column], places=8)

    def test_lidar_residual_is_not_zeroed_by_an_error_dependent_weight(self):
        source = (ROOT / "include/utils/ceresfunc.h").read_text(encoding="utf-8")
        estimator = (ROOT / "src/lio/Estimator.cpp").read_text(encoding="utf-8")
        self.assertNotIn("T _weight", source)
        self.assertNotIn("sqrt(\n              ceres::sqrt", source)
        self.assertIn("new ceres::HuberLoss", estimator)
        self.assertNotIn("loss_function = NULL", estimator)

        sigma = 0.02
        normalized = [error / sigma for error in (0.0, 0.1, 0.5, 1.0)]
        self.assertEqual(normalized, sorted(normalized))
        self.assertEqual(normalized[0], 0.0)
        self.assertTrue(all(value > 0.0 for value in normalized[1:]))

    def test_launch_files_expose_new_parameters_and_are_valid_xml(self):
        required = {
            "imu_time_offset",
            "imu_wait_timeout_ms",
            "max_imu_gap",
            "lidar_huber_delta",
            "lidar_outlier_threshold",
        }
        for launch_file in (ROOT / "launch").glob("*.launch"):
            tree = ET.parse(str(launch_file))
            names = {element.attrib.get("name") for element in tree.iter()}
            self.assertTrue(required.issubset(names), launch_file.name)


if __name__ == "__main__":
    unittest.main(verbosity=2)
