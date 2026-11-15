#!/usr/bin/env python3
"""robobase —— 三轮全向底盘 ROS 2 Humble 节点。

订阅:
    cmd_vel (geometry_msgs/Twist)
        linear.x  前进速度 (m/s)
        linear.y  左移速度 (m/s)
        angular.z 旋转角速度 (rad/s，逆时针为正)

发布:
    joint_states (sensor_msgs/JointState)
        name     = motor_<CAN ID>
        position = 电机位置 (rad)
        velocity = 电机转速 (rad/s)
        effort   = 电机转矩 (Nm)
        尚未收到反馈的电机对应项为 NaN

CAN:
    复用 can_motor_multi.MotorBank，motor_ids 的顺序与 wheel_angles_deg
    一一对应。底盘运动学见 omni_kinematics.py。

运行:
    python3 robobase.py
    python3 robobase.py --ros-args -p can_channel:=can0 \
        -p motor_ids:=[1,2,3] -p wheel_angles_deg:=[0.0,120.0,240.0]
"""

import math
import time

import can
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from can_motor_multi import MotorBank
from omni_kinematics import OmniDriveKinematics

DEFAULT_WHEEL_ANGLES_DEG = [0.0, 120.0, 240.0]


class Robobase(Node):

    def __init__(self):
        super().__init__("robobase")

        # ---------------- 参数 ----------------
        self.declare_parameter("wheel_radius", 0.041)          # 82mm 直径
        self.declare_parameter("base_radius", 0.300)           # 轮心到底盘中心
        self.declare_parameter("wheel_angles_deg",
                               DEFAULT_WHEEL_ANGLES_DEG)
        self.declare_parameter("wheel_direction", [1, 1, 1])   # 各电机安装方向 ±1
        self.declare_parameter("gear_ratio", 1.0)              # 电机/轮 转速比
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_interface", "socketcan")
        self.declare_parameter("motor_ids", [1, 2, 3])
        self.declare_parameter("control_rate", 50.0)           # Hz
        self.declare_parameter("feedback_timeout", 0.005)      # 每周期收反馈上限
        self.declare_parameter("max_motor_speed", 45.0)        # rad/s
        self.declare_parameter("cmd_vel_timeout", 0.5)         # 无指令安全停车

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.base_radius = self.get_parameter("base_radius").value
        angles_deg = list(self.get_parameter("wheel_angles_deg").value)
        self.wheel_direction = list(self.get_parameter("wheel_direction").value)
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.can_channel = self.get_parameter("can_channel").value
        self.can_interface = self.get_parameter("can_interface").value
        self.motor_ids = list(self.get_parameter("motor_ids").value)
        self.control_rate = self.get_parameter("control_rate").value
        self.feedback_timeout = self.get_parameter("feedback_timeout").value
        self.max_motor_speed = self.get_parameter("max_motor_speed").value
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value

        self._validate_parameters()

        self.kinematics = OmniDriveKinematics(
            self.wheel_radius,
            self.base_radius,
            [math.radians(a) for a in angles_deg],
        )

        # 目标电机转速 (rad/s)，与 motor_ids 顺序一致
        self._target_motor_speeds = [0.0] * len(self.motor_ids)
        self._last_cmd_time = None

        # ---------------- CAN ----------------
        self.bank = MotorBank(
            self.can_channel,
            self.motor_ids,
            self.can_interface,
        )
        self.bank.enable_all()
        time.sleep(0.05)
        self.get_logger().info(
            f"CAN {self.can_channel} ready, motors={self.motor_ids}"
        )
        self.get_logger().info(
            f"wheel_radius={self.wheel_radius:.4f} m, "
            f"base_radius={self.base_radius:.4f} m, "
            f"wheel_angles={angles_deg} deg"
        )

        # ---------------- ROS 接口 ----------------
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_cb, 10)
        self.create_timer(1.0 / self.control_rate, self._control_loop)

    # ---------------- 参数检查 ----------------
    def _validate_parameters(self):
        count = len(self.motor_ids)
        if count != 3:
            raise ValueError(f"需要 3 个电机 ID，当前为 {self.motor_ids}")
        if len(self.wheel_direction) != count:
            raise ValueError(
                f"wheel_direction 长度({len(self.wheel_direction)})"
                f"与电机数量({count})不一致"
            )
        if any(d not in (-1, 1) for d in self.wheel_direction):
            raise ValueError("wheel_direction 只能取 +1 或 -1")
        if len(set(self.motor_ids)) != count:
            raise ValueError(f"motor_ids 有重复: {self.motor_ids}")
        if self.wheel_radius <= 0 or self.base_radius <= 0:
            raise ValueError("wheel_radius 和 base_radius 必须大于 0")
        if self.gear_ratio <= 0:
            raise ValueError("gear_ratio 必须大于 0")
        if self.control_rate <= 0:
            raise ValueError("control_rate 必须大于 0")
        if not 0 < self.max_motor_speed <= 45.0:
            raise ValueError("max_motor_speed 必须在 (0, 45] rad/s 范围内")

        # wheel_angles_deg 在构造 OmniDriveKinematics 时检查长度

    # ---------------- cmd_vel 回调 ----------------
    def _cmd_vel_cb(self, msg: Twist):
        wheel_linear = self.kinematics.twist_to_wheel_linear_speeds(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )

        clipped = False
        for index, motor_id in enumerate(self.motor_ids):
            wheel_omega = wheel_linear[index] / self.wheel_radius
            motor_speed = (
                wheel_omega
                * self.gear_ratio
                * self.wheel_direction[index]
            )
            if motor_speed > self.max_motor_speed:
                motor_speed = self.max_motor_speed
                clipped = True
            elif motor_speed < -self.max_motor_speed:
                motor_speed = -self.max_motor_speed
                clipped = True
            self._target_motor_speeds[index] = motor_speed

        if clipped:
            self.get_logger().warn(
                f"电机转速超出 ±{self.max_motor_speed:.1f} rad/s，已限幅",
                throttle_duration_sec=1.0,
            )
        self._last_cmd_time = time.monotonic()

    # ---------------- 周期控制/反馈 ----------------
    def _control_loop(self):
        now = time.monotonic()
        if (self._last_cmd_time is None
                or now - self._last_cmd_time > self.cmd_vel_timeout):
            # 超时没有新指令，停车
            self._target_motor_speeds = [0.0] * len(self.motor_ids)

        speeds = {
            motor_id: self._target_motor_speeds[index]
            for index, motor_id in enumerate(self.motor_ids)
        }

        try:
            self.bank.set_speeds(speeds)
            self.bank.poll_feedback(self.feedback_timeout)
        except can.CanError as exc:
            self.get_logger().error(
                f"CAN 通信错误: {exc}", throttle_duration_sec=1.0
            )

        self._publish_joint_states()

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for motor_id in self.motor_ids:
            msg.name.append(f"motor_{motor_id}")
            state = self.bank.motors[motor_id].state
            if state is None:
                msg.position.append(math.nan)
                msg.velocity.append(math.nan)
                msg.effort.append(math.nan)
            else:
                msg.position.append(state["position"])
                msg.velocity.append(state["velocity"])
                msg.effort.append(state["torque"])

        self.joint_pub.publish(msg)

    # ---------------- 退出 ----------------
    def destroy_node(self):
        if hasattr(self, "bank"):
            try:
                self.bank.disable_all()
            except can.CanError as exc:
                self.get_logger().error(f"失能电机失败: {exc}")
            finally:
                self.bank.close()
                self.get_logger().info("CAN bus closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Robobase()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()