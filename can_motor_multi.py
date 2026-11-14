#!/usr/bin/env python3
"""
多电机 CAN 控制程序（基于 can_motor.py）

多个 DM2325 电机挂在同一条 CAN 总线上，CAN ID 从 1 开始依次递增：
    电机 1 -> 控制帧 ID 0x201
    电机 2 -> 控制帧 ID 0x202
    ...

所有电机共用一个 python-can Bus 实例，反馈报文先按仲裁 ID 匹配电机，
匹配不上时再按 D[0] 低 4 位的 controller_id 分发，因此同一总线上的
多路反馈不会串到别的电机。

示例:
    # 依次控制 ID=1、2、3 三个电机，速度均为 1.0 rad/s
    python can_motor_multi.py --count 3 --speed 1.0

    # 显式指定电机 ID 和各自的速度
    python can_motor_multi.py --motors 1 2 3 --speeds 1.0 -1.0 0.5

    # 运行 5 秒后自动停止
    python can_motor_multi.py --count 3 --speed 1.0 --duration 5
"""

import argparse
import time

import can

from can_motor import parse_can_feedback, speed_to_bytes

CONTROL_ID_BASE = 0x200   # 控制帧仲裁 ID = 0x200 + 电机 ID
ENABLE_CMD = 0xFC         # 使能命令（D[7]）
DISABLE_CMD = 0xFD        # 失能命令（D[7]）
SPEED_LIMIT = 45.0        # rad/s


def parse_args():
    parser = argparse.ArgumentParser(
        description="DM2325 multi-motor CAN control"
    )
    parser.add_argument(
        "--channel",
        default="can0",
        help="CAN 通道，默认 can0"
    )
    parser.add_argument(
        "--interface",
        default="socketcan",
        help="python-can 接口，默认 socketcan"
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="电机数量，CAN ID 从 1 开始连续递增（默认 1）"
    )
    parser.add_argument(
        "--motors",
        type=int,
        nargs="+",
        default=None,
        help="显式指定电机 CAN ID，例如 --motors 1 2 3（优先级高于 --count）"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.0,
        help="所有电机统一速度 rad/s，范围 -45~45"
    )
    parser.add_argument(
        "--speeds",
        type=float,
        nargs="+",
        default=None,
        help="按电机列表顺序逐个指定速度，例如 --speeds 1.0 -1.0 0.5"
    )
    parser.add_argument(
        "--period",
        type=float,
        default=1.0,
        help="控制周期（秒），默认 1.0"
    )
    parser.add_argument(
        "--feedback-timeout",
        type=float,
        default=0.9,
        help="每个周期内等待反馈的时长（秒），默认 0.9"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="运行时长（秒），0 表示一直运行"
    )
    return parser.parse_args()


def resolve_motor_ids(args) -> list:
    """确定要控制的电机 ID 列表，默认 1..count。"""
    if args.motors:
        motor_ids = list(args.motors)
    else:
        if args.count < 1:
            raise SystemExit(f"--count 必须 >= 1，当前为 {args.count}")
        motor_ids = list(range(1, args.count + 1))

    if len(set(motor_ids)) != len(motor_ids):
        raise SystemExit(f"电机 ID 有重复: {motor_ids}")
    for motor_id in motor_ids:
        if motor_id < 1 or CONTROL_ID_BASE + motor_id > 0x7FF:
            raise SystemExit(f"电机 ID {motor_id} 不合法")
    return motor_ids


def resolve_speeds(args, motor_ids: list) -> dict:
    """为每个电机确定目标速度（rad/s）。"""
    if args.speeds is None:
        return {motor_id: args.speed for motor_id in motor_ids}
    if len(args.speeds) != len(motor_ids):
        raise SystemExit(
            f"--speeds 数量({len(args.speeds)})与电机数量({len(motor_ids)})不一致"
        )
    return dict(zip(motor_ids, args.speeds))


class DM2325Motor:
    """单个 DM2325 电机，使用共享的 CAN 总线。"""

    def __init__(self, motor_id, bus):
        self.id = motor_id
        self.bus = bus
        self.state = None   # 最近一次解析出的反馈

    def _send(self, data):
        msg = can.Message(
            arbitration_id=CONTROL_ID_BASE + self.id,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)

    def enable_motor(self):
        data = [0xFF] * 8
        data[7] = ENABLE_CMD
        self._send(data)

    def disable_motor(self):
        data = [0xFF] * 8
        data[7] = DISABLE_CMD
        self._send(data)

    def send_speed(self, value):
        value = max(-SPEED_LIMIT, min(SPEED_LIMIT, float(value)))
        self._send(speed_to_bytes(value))


class MotorBank:
    """一组共享同一条 CAN 总线的 DM2325 电机。"""

    def __init__(self, channel, motor_ids, interface="socketcan"):
        self.bus = can.interface.Bus(
            channel=channel,
            interface=interface
        )
        self.motors = {motor_id: DM2325Motor(motor_id, self.bus)
                       for motor_id in motor_ids}

    @property
    def ids(self) -> list:
        return list(self.motors)

    def enable_all(self):
        for motor in self.motors.values():
            motor.enable_motor()

    def disable_all(self):
        for motor in self.motors.values():
            motor.disable_motor()

    def set_speeds(self, speeds: dict):
        for motor_id, motor in self.motors.items():
            motor.send_speed(speeds[motor_id])

    def _route(self, msg):
        """判断一条反馈报文属于哪个电机，无法判断时返回 None。"""
        # 优先用仲裁 ID（部分固件反馈帧仲裁 ID 就是电机 ID）
        if msg.arbitration_id in self.motors:
            return msg.arbitration_id
        # 再用 D[0] 低 4 位的 controller_id
        if len(msg.data) >= 1:
            ctrl_id = msg.data[0] & 0x0F
            if ctrl_id in self.motors:
                return ctrl_id
        # 单电机时兼容基类做法：反馈帧仲裁 ID 为 0x00
        if len(self.motors) == 1:
            return next(iter(self.motors))
        return None

    def poll_feedback(self, timeout: float) -> set:
        """在 timeout 秒内接收并分发反馈，返回本轮有反馈的电机 ID 集合。

        所有电机都收到反馈后立即返回，只有收不齐时才等满 timeout。
        """
        deadline = time.monotonic() + timeout
        received = set()
        while len(received) < len(self.motors):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            msg = self.bus.recv(timeout=remaining)
            if msg is None:
                break
            motor_id = self._route(msg)
            if motor_id is None:
                continue
            try:
                state = parse_can_feedback(msg.data)
            except ValueError:
                continue
            self.motors[motor_id].state = state
            received.add(motor_id)
        return received

    def close(self):
        self.bus.shutdown()


def format_states(bank: MotorBank, received: set) -> str:
    """把各电机最近一次反馈格式化成表格。"""
    lines = [
        f"{'ID':>3} {'位置(rad)':>10} {'速度(rad/s)':>12} "
        f"{'扭矩(Nm)':>10} {'MOS':>5} {'线圈':>5}  状态"
    ]
    for motor_id in bank.ids:
        state = bank.motors[motor_id].state
        if state is None:
            lines.append(f"{motor_id:>3} {'--':>10} {'--':>12} {'--':>10} "
                         f"{'--':>5} {'--':>5}  无反馈")
            continue
        suffix = "" if motor_id in received else " (本轮无反馈)"
        lines.append(
            f"{motor_id:>3} {state['position']:>10.3f} {state['velocity']:>12.3f} "
            f"{state['torque']:>10.3f} {state['t_mos']:>5d} {state['t_rotor']:>5d}  "
            f"{state['error_str']}{suffix}"
        )
    return "\n".join(lines)


def main():
    args = parse_args()
    motor_ids = resolve_motor_ids(args)
    speeds = resolve_speeds(args, motor_ids)

    bank = MotorBank(args.channel, motor_ids, args.interface)
    print(
        f"DM2325 multi-motor start: channel={args.channel}, "
        f"motors={motor_ids}, "
        f"speeds={[speeds[motor_id] for motor_id in motor_ids]}"
    )

    try:
        bank.enable_all()
        time.sleep(0.05)
        start = time.monotonic()
        elapsed = 0

        while True:
            iter_start = time.monotonic()

            # 控制：同一周期内依次给每个电机下发速度
            bank.set_speeds(speeds)

            # 读取：在本周期内收集所有电机的反馈
            received = bank.poll_feedback(args.feedback_timeout)
            print(f"\n[{time.strftime('%H:%M:%S')}] Elapsed:{elapsed}s")
            print(format_states(bank, received))

            if args.duration > 0 and time.monotonic() - start >= args.duration:
                break

            elapsed = time.monotonic() - iter_start

            time.sleep(max(0.0, args.period - elapsed))

    except KeyboardInterrupt:
        print("\nExit by user")
    finally:
        bank.disable_all()
        bank.close()
        print("CAN bus closed")


if __name__ == "__main__":
    main()