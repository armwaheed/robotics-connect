#!/usr/bin/env python3
"""Offline tests for the robot-agnostic policy-deploy ladder, on MockRobotIO. No hardware.

Run: ``python3 test_policy_deploy.py``
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import policy_deploy as pd  # noqa: E402

NAMES = ["left_hip_pitch_joint", "left_knee_joint", "right_shoulder_pitch_joint"]
DEFAULT = np.array([-0.1, 0.3, 0.3])
CMD = [0.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]


def _contract():
    return pd.DeployContract(
        action_joint_names=list(NAMES),
        action_scale=np.array([0.5, 0.5, 0.5]),
        action_offset=DEFAULT.copy(),
        joint_pos_default=DEFAULT.copy(),
        obs_term_order=["base_lin_vel", "base_ang_vel", "projected_gravity",
                        "hand_target", "joint_pos", "joint_vel", "actions"],
        obs_total_dim=3 + 3 + 3 + 7 + 3 + 3 + 3,
        control_hz=50.0,
        gains={n: (40.0, 2.0) for n in NAMES},
    )


def _hold_policy(_obs):
    return np.zeros(len(NAMES))


def _no_sleep():
    def _ns(_dt):
        return None
    pd._sleep = _ns


# ── tests ─────────────────────────────────────────────────────────────────────
def test_obs_dim_and_gravity():
    c = _contract()
    io = pd.MockRobotIO(NAMES, DEFAULT)
    dep = pd.PolicyDeploy(c, _hold_policy, io)
    obs, parts = dep.obs.build(io.read_state(), CMD, np.zeros(c.n))
    assert obs.shape[0] == c.obs_total_dim
    assert np.allclose(parts["projected_gravity"], [0, 0, -1], atol=1e-6)


def test_targets_offset_scale():
    c = _contract()
    io = pd.MockRobotIO(NAMES, DEFAULT)
    dep = pd.PolicyDeploy(c, _hold_policy, io)
    tq = dep.targets(np.array([1.0, 0.0, -1.0]))          # offset + 0.5*action
    assert abs(tq["left_hip_pitch_joint"] - (-0.1 + 0.5)) < 1e-9
    assert abs(tq["right_shoulder_pitch_joint"] - (0.3 - 0.5)) < 1e-9


def test_offline_publishes_nothing():
    c = _contract()
    io = pd.MockRobotIO(NAMES, DEFAULT)
    dep = pd.PolicyDeploy(c, _hold_policy, io)
    out = dep.run_offline(CMD, steps=3, log=lambda *_: None)
    assert len(out) == 3 and len(io.published) == 0 and io.damps == 0


def test_partial_publishes_weight_then_damps():
    _no_sleep()
    c = _contract()
    io = pd.MockRobotIO(NAMES, DEFAULT)
    dep = pd.PolicyDeploy(c, _hold_policy, io)
    dep.run_partial(CMD, subset=["right_shoulder_pitch_joint"], seconds=0.1,
                    clamp={"right_shoulder_pitch_joint": (-0.5, 0.5)}, log=lambda *_: None)
    assert io.published, "partial must publish"
    assert all(w is not None for _, w in io.published), "partial must use the overlay weight"
    assert io.damps > 0, "partial must damp on exit (SafeStop)"


def test_partial_respects_clamp():
    _no_sleep()
    c = _contract()
    io = pd.MockRobotIO(NAMES, DEFAULT)
    def big(_obs):                                          # drive shoulder hard past the clamp
        return np.array([0.0, 0.0, 10.0])
    dep = pd.PolicyDeploy(c, big, io)
    dep.run_partial(CMD, subset=["right_shoulder_pitch_joint"], seconds=0.3,
                    clamp={"right_shoulder_pitch_joint": (-0.5, 0.5)}, log=lambda *_: None)
    last = io.published[-1][0]["right_shoulder_pitch_joint"]
    assert last <= 0.5 + 1e-6, f"clamp violated: {last}"


def test_whole_refuses_unverified_release():
    _no_sleep()
    c = _contract()
    io = pd.MockRobotIO(NAMES, DEFAULT, can_release=False)
    dep = pd.PolicyDeploy(c, _hold_policy, io)
    dep.run_whole(CMD, seconds=0.1, log=lambda *_: None)
    assert not io.published, "must not take over without a verified vendor release"


def test_whole_runs_full_targets_then_damps():
    _no_sleep()
    c = _contract()
    io = pd.MockRobotIO(NAMES, DEFAULT, can_release=True)
    dep = pd.PolicyDeploy(c, _hold_policy, io)
    dep.run_whole(CMD, seconds=0.05, settle_s=0.05, blend_s=0.05, log=lambda *_: None)
    assert io.released and io.published and io.damps > 0
    assert all(w is None for _, w in io.published), "whole-body uses full targets, no overlay weight"


def test_abort_damps_and_stops():
    _no_sleep()
    c = _contract()
    io = pd.MockRobotIO(NAMES, DEFAULT, abort_after=2)
    dep = pd.PolicyDeploy(c, _hold_policy, io)
    dep.run_whole(CMD, seconds=5.0, settle_s=0.0, blend_s=0.0, log=lambda *_: None)
    assert io.damps > 0, "abort must damp"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"policy_deploy: {len(tests)}/{len(tests)} passed")
