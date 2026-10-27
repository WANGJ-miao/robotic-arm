"""Planar nested motion. Angles are radians; vectors use the containing frame."""
from dataclasses import dataclass
import numpy as np

LENGTHS = (0.40, 0.35, 0.20)
POINTS = ("end", "com1", "com2", "com3")


@dataclass
class PointState:
    position     : np.ndarray
    velocity     : np.ndarray
    acceleration : np.ndarray


@dataclass
class FrameState:
    origin       : np.ndarray
    velocity     : np.ndarray
    acceleration : np.ndarray
    angle        : float
    omega        : float
    alpha        : float


@dataclass
class Composition:
    state : PointState
    ae    : np.ndarray
    ar    : np.ndarray
    aC    : np.ndarray


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]])


def perpendicular(v):
    """Unit z vector crossed with a planar vector."""
    return np.array([-v[1], v[0]])


def compose(frame, point):
    """Map a local point to the parent frame; all outputs use parent axes.

    frame origin derivatives are expressed in parent axes. point derivatives
    are relative derivatives in local axes. This function also applies when
    the parent is a moving frame: outputs then remain relative to that parent.
    """
    R = rotation(frame.angle)
    r, vr = R @ point.position, R @ point.velocity
    ae = (frame.acceleration + frame.alpha * perpendicular(r)
          - frame.omega**2 * r)
    ar = R @ point.acceleration
    aC = 2 * frame.omega * perpendicular(vr)
    state = PointState(frame.origin + r,
                       frame.velocity + frame.omega * perpendicular(r) + vr,
                       ae + ar + aC)
    return Composition(state, ae, ar, aC)


def carrier_k1(t, omega=0.4):
    return FrameState(np.zeros(2), np.zeros(2), np.zeros(2), omega*t, omega, 0.0)


def carrier_k2(t, heave=0.15, heave_period=4.0,
               roll=np.deg2rad(10), roll_period=5.0):
    w, u = 2*np.pi/heave_period, 2*np.pi/roll_period
    return FrameState(np.array([0., heave*np.sin(w*t)]),
                      np.array([0., heave*w*np.cos(w*t)]),
                      np.array([0., -heave*w*w*np.sin(w*t)]),
                      roll*np.sin(u*t), roll*u*np.cos(u*t),
                      -roll*u*u*np.sin(u*t))


def joint_motion(t):
    q0 = np.deg2rad([30., -20., 15.])
    amplitude = np.deg2rad([20., 15., 10.])
    w = 2*np.pi/np.array([4., 3., 2.5])
    return (q0 + amplitude*np.sin(w*t), amplitude*w*np.cos(w*t),
            -amplitude*w*w*np.sin(w*t))


def arm_points(q, qd, qdd, lengths=LENGTHS):
    """Return base-relative point states and each local composition.

    Start with a fixed point in its own link, then compose inward through
    every ancestor. No position differentiation or Jacobian is used.
    """
    states, stages = {}, {}
    for name, link, fraction in (("end", 2, 1.), ("com1", 0, .5),
                                  ("com2", 1, .5), ("com3", 2, .5)):
        point = PointState(np.array([fraction*lengths[link], 0.]),
                           np.zeros(2), np.zeros(2))
        stages[name] = []
        for i in range(link, -1, -1): # range(start, end, step)
            origin = np.array([lengths[i-1] if i else 0., 0.])
            frame = FrameState(origin, np.zeros(2), np.zeros(2), q[i], qd[i], qdd[i])
            result = compose(frame, point)
            stages[name].append((f"S{i+2}_to_S{i+1}", result)) # "S" stands for system
            point = result.state
        states[name] = point
    return states, stages       # stages: all the intermidiate calculation steps. states: the final outcome, the states of end and link1,2,3 points.


def evaluate(t, carrier, lengths=LENGTHS, motion=joint_motion):
    relative, stages = arm_points(*motion(t), lengths) # "relative" means relative to the base
    absolute = {name: compose(carrier(t), point) for name, point in relative.items()}
    return relative, absolute, stages
