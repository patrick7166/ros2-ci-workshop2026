# Copyright 2026 Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Pure velocity-limiting logic, deliberately free of any ROS imports.

Keeping the decision-making in a plain Python module is what makes this
package cheap to test in CI: no DDS traffic, no simulator, no GPU, no
hardware.  The ROS node in :mod:`turtle_guard.velocity_guard_node` is a thin
wrapper that only moves data in and out of this function.
"""


class UnsafeLimitError(ValueError):
    """Raised when a caller asks for a negative or non-finite speed limit."""


def clamp(value: float, limit: float) -> float:
    """
    Clamp ``value`` into the closed interval ``[-limit, +limit]``.

    :param value: the requested speed, in m/s or rad/s.
    :param limit: the maximum magnitude allowed; must be >= 0.
    :returns: the requested speed, saturated at ``limit``.
    :raises UnsafeLimitError: if ``limit`` is negative.
    """
    if limit < 0.0:
        raise UnsafeLimitError(f'speed limit must be >= 0, got {limit}')
    return max(-limit, min(limit, value))


def clamp_twist(linear_x, angular_z, linear_limit, angular_limit):
    """
    Clamp a 2D differential-drive command.

    :param linear_x: requested forward speed in m/s.
    :param angular_z: requested yaw rate in rad/s.
    :param linear_limit: maximum forward speed magnitude in m/s.
    :param angular_limit: maximum yaw rate magnitude in rad/s.
    :returns: a ``(linear_x, angular_z)`` tuple, both saturated.
    """
    return (
        clamp(linear_x, linear_limit),
        clamp(angular_z, angular_limit),
    )


def is_stale(now_sec: float, last_msg_sec: float, timeout_sec: float) -> bool:
    """
    Return ``True`` when the last command is older than ``timeout_sec``.

    A stale command stream is the classic reason a robot keeps driving into a
    wall after the teleop laptop dies, so the guard node stops the robot when
    this returns ``True``.
    """
    if timeout_sec <= 0.0:
        return False
    return (now_sec - last_msg_sec) > timeout_sec
