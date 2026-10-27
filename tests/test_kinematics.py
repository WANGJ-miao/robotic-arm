import unittest
import numpy as np

from model.kinematics import (FrameState, PointState, arm_points, carrier_k1,
                              compose, evaluate, joint_motion, perpendicular)
from model.basic_model import forward_kinematics


class KinematicsTests(unittest.TestCase):
    def test_fixed_base_matches_geometry_and_jacobian(self):
        from model.basic_model import Jacobbin_end, Jacobbin_1, Jacobbin_2, Jacobbin_3
        lengths = (.4, .35, .2)
        q, qd, qdd = joint_motion(.73)
        states, _ = arm_points(q, qd, qdd)
        positions = forward_kinematics(q, lengths)[2:]
        for name, p, jacobian in zip(("end", "com1", "com2", "com3"), positions,
                                     (Jacobbin_end, Jacobbin_1, Jacobbin_2, Jacobbin_3)):
            np.testing.assert_allclose(states[name].position, p, atol=1e-14)
            np.testing.assert_allclose(states[name].velocity, jacobian(q, lengths) @ qd, atol=1e-14)

    def test_stationary_arm_rotating_carrier(self):
        motion = lambda t: (np.array([.3, -.4, .2]), np.zeros(3), np.zeros(3))
        _, absolute, _ = evaluate(.8, carrier_k1, motion=motion)
        for result in absolute.values():
            np.testing.assert_allclose(result.state.velocity, .4*perpendicular(result.state.position))
            np.testing.assert_allclose(result.state.acceleration, -.16*result.state.position)
            np.testing.assert_allclose(result.ar, 0., atol=1e-14)
            np.testing.assert_allclose(result.aC, 0., atol=1e-14)

    def test_pure_translation(self):
        frame = FrameState(np.array([1., 2.]), np.array([.5, -.2]),
                           np.array([.1, .3]), 0., 0., 0.)
        point = PointState(np.array([.2, .3]), np.array([.4, .5]), np.array([.6, .7]))
        result = compose(frame, point)
        np.testing.assert_allclose(result.state.position, [1.2, 2.3])
        np.testing.assert_allclose(result.state.velocity, [.9, .3])
        np.testing.assert_allclose(result.state.acceleration, [.7, 1.])
        np.testing.assert_allclose(result.aC, 0.)

    def test_rotating_radial_slide_has_coriolis(self):
        frame = FrameState(np.zeros(2), np.zeros(2), np.zeros(2), 0., 2., 3.)
        point = PointState(np.array([4., 0.]), np.array([5., 0.]), np.array([6., 0.]))
        result = compose(frame, point)
        np.testing.assert_allclose(result.ae, [-16., 12.])
        np.testing.assert_allclose(result.aC, [0., 20.])
        np.testing.assert_allclose(result.state.acceleration, [-10., 32.])

    def test_all_static(self):
        states, _ = arm_points([.2, .4, -.3], np.zeros(3), np.zeros(3))
        for state in states.values():
            np.testing.assert_allclose(state.velocity, 0.)
            np.testing.assert_allclose(state.acceleration, 0.)


if __name__ == "__main__":
    unittest.main()
