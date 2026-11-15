#!/usr/bin/env python3
"""三轮全向底盘（kiwi drive）运动学。

坐标约定遵循 ROS REP-103：
    x 向前，y 向左，z 向上；
    角速度 wz > 0 表示绕 +z 逆时针旋转。

轮子安装约定：
    轮子 i 位于方位角 theta_i（从 +x 轴逆时针测量），
    轮轴指向底盘中心，驱动方向为圆周切向：
        t_i = (-sin(theta_i), cos(theta_i))
    因此轮子线速度：
        v_i = -vx*sin(theta_i) + vy*cos(theta_i) + wz*base_radius
"""

import math

WHEEL_COUNT = 3


def _invert_3x3(matrix):
    """纯 Python 3x3 矩阵求逆。"""
    (a, b, c), (d, e, f), (g, h, i) = matrix
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError("运动学矩阵不可逆，请检查轮子角度配置")

    inverse = (
        ((e * i - f * h), (c * h - b * i), (b * f - c * e)),
        ((f * g - d * i), (a * i - c * g), (c * d - a * f)),
        ((d * h - e * g), (b * g - a * h), (a * e - b * d)),
    )
    return tuple(tuple(value / det for value in row) for row in inverse)


class OmniDriveKinematics:
    """三轮全向底盘正/逆运动学。"""

    def __init__(self, wheel_radius, base_radius, wheel_angles):
        if len(wheel_angles) != WHEEL_COUNT:
            raise ValueError(
                f"需要 {WHEEL_COUNT} 个轮子角度，当前为 {len(wheel_angles)}"
            )
        self.wheel_radius = float(wheel_radius)
        self.base_radius = float(base_radius)
        self.wheel_angles = tuple(float(a) for a in wheel_angles)

        # 正运动学矩阵 A：v = A * [vx, vy, wz]^T
        self._forward = tuple(
            (-math.sin(a), math.cos(a), self.base_radius)
            for a in self.wheel_angles
        )
        self._inverse = _invert_3x3(self._forward)

    def twist_to_wheel_linear_speeds(self, vx, vy, wz):
        """底盘速度 (m/s, m/s, rad/s) -> 各轮线速度 (m/s)。"""
        return tuple(
            -vx * math.sin(a) + vy * math.cos(a) + wz * self.base_radius
            for a in self.wheel_angles
        )

    def twist_to_wheel_angular_speeds(self, vx, vy, wz):
        """底盘速度 -> 各轮角速度 (rad/s)。"""
        return tuple(
            speed / self.wheel_radius
            for speed in self.twist_to_wheel_linear_speeds(vx, vy, wz)
        )

    def wheel_angular_speeds_to_twist(self, wheel_speeds):
        """各轮角速度 (rad/s) -> 底盘速度 (vx, vy, wz)。"""
        if len(wheel_speeds) != WHEEL_COUNT:
            raise ValueError(
                f"需要 {WHEEL_COUNT} 个轮速，当前为 {len(wheel_speeds)}"
            )
        wheel_linear = [w * self.wheel_radius for w in wheel_speeds]
        inv = self._inverse
        vx = sum(inv[0][j] * wheel_linear[j] for j in range(WHEEL_COUNT))
        vy = sum(inv[1][j] * wheel_linear[j] for j in range(WHEEL_COUNT))
        wz = sum(inv[2][j] * wheel_linear[j] for j in range(WHEEL_COUNT))
        return vx, vy, wz