#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 键盘遥控节点：按键按下时按设定速度发布 cmd_vel，松开后发布 0。

按键映射
--------
    方向键 ↑ / ↓        vx = +设定线速度 / -设定线速度
    方向键 ← / →        vy = +设定线速度 / -设定线速度（ROS 约定 +y 朝左）
    q / e               wz = +设定角速度 / -设定角速度
    w / s               设定线速度 +0.1 / -0.1  (m/s)
    a / d               设定角速度 +0.1 / -0.1  (rad/s)
    Esc                 退出并发布 0（Ctrl-C 也可以）

输入源（Ubuntu 22.04 Server，无图形界面）
-----------------------------------------
终端里拿不到“松开”事件，因此提供两种输入源，自动选择：

  * tty（SSH / 本地终端都能用，默认）
        在前台终端里用 raw 模式读按键。方向键是 ESC 序列
        （\\x1b[A/B/C/D 或 \\x1bOA/B/C/D）。由于终端没有松开事件，
        改为观察“键盘自动重复”：一段时间内收不到同一个键的重复，
        就认为已松开并发布 0。
          - 首次按键后宽限 tty_repeat_grace（默认 0.7 s，覆盖系统约
            500 ms 的重复延迟），避免长按中途误停；
          - 识别出重复节奏后，约 2.5 倍重复间隔（默认下限 0.12 s）
            内收不到即停，松手到车停有 0.1~0.7 s 的延迟。
        前提：SSH 客户端 / 终端开启键盘自动重复（默认开启）。

  * evdev（USB 键盘直接接在 RDK X5 上）
        真正的按下/松开事件，松手立刻停，没有启发式延迟。需要读权限：

            sudo usermod -aG input $USER    # 重新登录后生效
            python3 cmd_vel_keyboard.py --ros-args -p keyboard_backend:=evdev

用法
----
    source /opt/ros/humble/setup.bash
    python3 cmd_vel_keyboard.py
    python3 cmd_vel_keyboard.py --ros-args -p linear_speed:=0.8 -p topic:=cmd_vel
    ros2 topic echo /cmd_vel

参数（ros2 param / --ros-args -p 均可）
--------------------------------------
    topic               发布话题，默认 cmd_vel
    linear_speed        初始线速度设定，默认 0.5 m/s
    angular_speed       初始角速度设定，默认 1.0 rad/s
    speed_step          每次按键的调整步长，默认 0.1
    max_linear_speed    线速度上限，默认 2.0 m/s
    max_angular_speed   角速度上限，默认 4.0 rad/s
    publish_rate        按住按键时的重复发布频率，默认 20 Hz
    keyboard_backend    auto / tty / evdev，默认 auto
    tty_repeat_grace    首次按键的保持宽限，默认 0.7 s
    tty_release_timeout 识别重复节奏后的最小判定间隔，默认 0.12 s

按住按键时节点会以 publish_rate 持续发布同一速度；松开最后一个运动键后
补发一次 0。Ctrl-C、Esc、异常退出时都会补发 0。
"""

import os
import select
import sys
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# 会产生速度的按键
MOTION_KEYS = frozenset(("up", "down", "left", "right", "q", "e"))
# 只修改速度设定的按键
ADJUST_KEYS = frozenset(("w", "s", "a", "d"))
KNOWN_KEYS = MOTION_KEYS | ADJUST_KEYS | {"esc"}

# 终端方向键转义序列
_KEY_SEQUENCES = {
    b"\x1b[A": "up",
    b"\x1b[B": "down",
    b"\x1b[C": "right",
    b"\x1b[D": "left",
    b"\x1bOA": "up",
    b"\x1bOB": "down",
    b"\x1bOC": "right",
    b"\x1bOD": "left",
}
# 普通字符键
_CHAR_KEYS = {
    b"q": "q",
    b"e": "e",
    b"w": "w",
    b"s": "s",
    b"a": "a",
    b"d": "d",
}
# 单独的 ESC 与方向键序列前缀重叠，稍等一点再判定
_ESC_TIMEOUT = 0.04

_EVDEV_NAMES = {
    "KEY_UP": "up",
    "KEY_DOWN": "down",
    "KEY_LEFT": "left",
    "KEY_RIGHT": "right",
    "KEY_Q": "q",
    "KEY_E": "e",
    "KEY_W": "w",
    "KEY_S": "s",
    "KEY_A": "a",
    "KEY_D": "d",
    "KEY_ESC": "esc",
}


class _TtySource:
    """前台终端 raw 模式读键；无松开事件，由节点按自动重复节奏判定。"""

    uses_hold_timeout = True

    def __init__(self, on_press):
        self._on_press = on_press
        self._fd = None
        self._saved_attrs = None
        self._thread = None
        self._stopped = threading.Event()

    def start(self):
        if not sys.stdin.isatty():
            raise RuntimeError("标准输入不是终端，请直接在前台终端里运行本节点")
        try:
            self._fd = sys.stdin.fileno()
        except (AttributeError, ValueError, OSError) as exc:
            raise RuntimeError("标准输入不可用：%s" % exc) from exc

        import termios
        import tty as tty_module

        self._saved_attrs = termios.tcgetattr(self._fd)
        # cbreak：关闭行缓冲和回显，但保留 Ctrl-C（ISIG）
        tty_module.setcbreak(self._fd)
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stopped.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._saved_attrs is not None:
            import termios

            try:
                termios.tcsetattr(
                    self._fd, termios.TCSADRAIN, self._saved_attrs
                )
            except (termios.error, OSError):
                pass
            self._saved_attrs = None

    @staticmethod
    def _parse(buffer):
        """把缓冲区解析成按键名列表，返回 (按键列表, 剩余未完成的字节)。"""
        names = []
        while buffer:
            sequence = next(
                (seq for seq in _KEY_SEQUENCES if buffer.startswith(seq)), None
            )
            if sequence is not None:
                names.append(_KEY_SEQUENCES[sequence])
                buffer = buffer[len(sequence):]
                continue
            if buffer[0] == 0x1B:
                # 可能是还没收全的方向键序列，或单独的 ESC，交给超时判定
                if any(seq.startswith(buffer) for seq in _KEY_SEQUENCES):
                    return names, buffer
                # 其它未知转义序列（如某些终端的鼠标事件），丢弃首字节
                buffer = buffer[1:]
                continue
            byte = buffer[:1]
            buffer = buffer[1:]
            name = _CHAR_KEYS.get(byte.lower())
            if name is not None:
                names.append(name)
        return names, buffer

    def _read_loop(self):
        buffer = b""
        pending_since = None
        while not self._stopped.is_set():
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.05)
            except (OSError, ValueError):
                break
            if self._stopped.is_set():
                break

            if ready:
                try:
                    data = os.read(self._fd, 64)
                except OSError:
                    break
                if not data:  # 终端关闭
                    break
                buffer += data

            names, buffer = self._parse(buffer)
            for name in names:
                self._on_press(name)

            now = time.monotonic()
            if not buffer:
                pending_since = None
            elif pending_since is None:
                pending_since = now
            elif now - pending_since >= _ESC_TIMEOUT:
                if buffer == b"\x1b":
                    self._on_press("esc")
                buffer = b""
                pending_since = None


class _EvdevSource:
    """直接读取 /dev/input/event*，真正的按下/松开事件（需 input 组权限）。"""

    uses_hold_timeout = False

    def __init__(self, on_press, on_release):
        from evdev import ecodes

        self._ecodes = ecodes
        self._on_press = on_press
        self._on_release = on_release
        self._code_names = {
            getattr(ecodes, attr): name
            for attr, name in _EVDEV_NAMES.items()
            if hasattr(ecodes, attr)
        }
        self._devices = []
        self._threads = []
        self._stopped = threading.Event()

    def _open_devices(self):
        import evdev

        ecodes = self._ecodes
        devices = []
        denied = []
        for path in evdev.list_devices():
            try:
                device = evdev.InputDevice(path)
            except (PermissionError, OSError):
                denied.append(path)
                continue
            keys = device.capabilities().get(ecodes.EV_KEY, [])
            # 带方向键的输入设备才视为键盘，避免去读鼠标等设备
            if ecodes.KEY_UP in keys and ecodes.KEY_A in keys:
                devices.append(device)
            else:
                device.close()

        if denied and not devices:
            raise RuntimeError(
                "无权限读取 %s，请把当前用户加入 input 组后重新登录 "
                "(sudo usermod -aG input $USER)，或使用 sudo 运行"
                % ", ".join(denied)
            )
        if not devices:
            raise RuntimeError("未找到键盘设备（/dev/input 下没有带方向键的输入设备）")
        return devices

    def _read_loop(self, device):
        ecodes = self._ecodes
        try:
            for event in device.read_loop():
                if self._stopped.is_set():
                    break
                if event.type != ecodes.EV_KEY:
                    continue
                name = self._code_names.get(event.code)
                if name is None:
                    continue
                if event.value == 1:
                    self._on_press(name)
                elif event.value == 0:
                    self._on_release(name)
                # event.value == 2 是自动重复，忽略
        except OSError:
            pass
        finally:
            try:
                device.close()
            except OSError:
                pass

    def start(self):
        self._devices = self._open_devices()
        for device in self._devices:
            thread = threading.Thread(
                target=self._read_loop, args=(device,), daemon=True
            )
            thread.start()
            self._threads.append(thread)

    def stop(self):
        self._stopped.set()
        for device in self._devices:
            try:
                device.close()
            except OSError:
                pass
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
        self._devices.clear()


def start_keyboard_source(backend, on_press, on_release, logger):
    """创建并启动键盘输入源，失败时自动尝试另一种。"""
    backend = (backend or "auto").strip().lower()
    if backend == "auto":
        # 前台终端优先 tty（SSH 也能用）；否则（systemd/launch 启动）用 evdev
        order = ["tty", "evdev"] if sys.stdin.isatty() else ["evdev", "tty"]
    elif backend in ("tty", "evdev"):
        order = [backend]
    else:
        raise RuntimeError(
            "未知的 keyboard_backend：%s（可选 auto / tty / evdev）" % backend
        )

    errors = []
    for name in order:
        try:
            source = (
                _TtySource(on_press)
                if name == "tty"
                else _EvdevSource(on_press, on_release)
            )
            source.start()
            logger.info("键盘输入源：%s" % name)
            return source
        except Exception as exc:  # noqa: BLE001 - 需要回退到另一种输入源
            errors.append("%s: %s" % (name, exc))
            logger.warning("键盘输入源 %s 不可用：%s" % (name, exc))

    raise RuntimeError("没有可用的键盘输入源 -> " + "; ".join(errors))


class KeyboardTeleopNode(Node):
    """监听键盘，把按键状态转换为 geometry_msgs/Twist 并发布。"""

    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("topic", "cmd_vel")
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("speed_step", 0.1)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_speed", 4.0)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("keyboard_backend", "auto")
        self.declare_parameter("tty_repeat_grace", 0.7)
        self.declare_parameter("tty_release_timeout", 0.12)

        topic = self._param("topic")
        self._step = abs(float(self._param("speed_step")))
        self._max_linear = abs(float(self._param("max_linear_speed")))
        self._max_angular = abs(float(self._param("max_angular_speed")))
        self._linear_speed = self._clamp(
            float(self._param("linear_speed")), self._max_linear
        )
        self._angular_speed = self._clamp(
            float(self._param("angular_speed")), self._max_angular
        )
        publish_rate = max(1.0, float(self._param("publish_rate")))
        self._repeat_grace = max(0.05, float(self._param("tty_repeat_grace")))
        self._release_timeout = max(
            0.02, float(self._param("tty_release_timeout"))
        )

        # 监听线程只往队列里塞事件，状态变更和发布都在定时器（ROS 线程）里做
        self._lock = threading.Lock()
        self._events = deque()
        self._held = set()
        self._deadlines = {}    # tty 模式：键名 -> 视为松开的截止时间
        self._last_seen = {}    # tty 模式：键名 -> 上次收到事件的时刻
        self._repeat_seen = {}  # tty 模式：键名 -> 已观察到的重复次数
        self._use_hold_timeout = False
        self._last_twist = (0.0, 0.0, 0.0)
        self._quit = False
        self._source = None

        self._publisher = self.create_publisher(Twist, topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            "开始键盘遥控：方向键控制 vx/vy，q/e 控制 wz，"
            "w/s 调线速度 ±%.2f m/s，a/d 调角速度 ±%.2f rad/s，Esc 退出"
            % (self._step, self._step)
        )
        self._log_speeds()

    # ------------------------------------------------------------------ 参数
    def _param(self, name):
        return self.get_parameter(name).value

    def _clamp(self, value, limit):
        return round(max(0.0, min(limit, value)), 6)

    def _log_speeds(self):
        self.get_logger().info(
            "当前设定：线速度 %.2f m/s，角速度 %.2f rad/s"
            % (self._linear_speed, self._angular_speed)
        )

    # ------------------------------------------------------------ 键盘输入源
    def start_keyboard(self):
        backend = self._param("keyboard_backend")
        self._source = start_keyboard_source(
            backend, self._enqueue_press, self._enqueue_release, self.get_logger()
        )
        self._use_hold_timeout = getattr(
            self._source, "uses_hold_timeout", False
        )

    def stop_keyboard(self):
        if self._source is not None:
            self._source.stop()
            self._source = None

    def _enqueue_press(self, name):
        with self._lock:
            self._events.append(("press", name, time.monotonic()))

    def _enqueue_release(self, name):
        with self._lock:
            self._events.append(("release", name, time.monotonic()))

    # -------------------------------------------------------------- 状态处理
    def _on_press(self, name, stamp):
        if name == "esc":
            self._quit = True
            return
        if name in self._held:
            # tty：这是键盘自动重复，刷新保持时间；evdev 不会重复送来 press
            if self._use_hold_timeout:
                self._refresh_hold(name, stamp)
            return
        self._held.add(name)
        if self._use_hold_timeout:
            self._last_seen[name] = stamp
            self._repeat_seen[name] = 0
            self._deadlines[name] = stamp + self._repeat_grace
        if name in ADJUST_KEYS:
            self._adjust(name)

    def _refresh_hold(self, name, stamp):
        previous = self._last_seen.get(name, stamp)
        repeats = self._repeat_seen.get(name, 0) + 1
        self._last_seen[name] = stamp
        self._repeat_seen[name] = repeats
        if repeats >= 2:
            # 已认出自动重复节奏，按 2.5 倍间隔判定松开
            timeout = max(self._release_timeout, 2.5 * (stamp - previous))
        else:
            # 终端首次重复延迟通常约 500 ms，先给足宽限，避免长按误停
            timeout = self._repeat_grace
        self._deadlines[name] = stamp + timeout

    def _on_release(self, name, stamp):
        self._clear_key(name)

    def _clear_key(self, name):
        self._held.discard(name)
        self._deadlines.pop(name, None)
        self._last_seen.pop(name, None)
        self._repeat_seen.pop(name, None)

    def _expire_holds(self, now):
        for name, deadline in list(self._deadlines.items()):
            if deadline <= now:
                self._clear_key(name)

    def _adjust(self, name):
        if name == "w":
            self._linear_speed = self._clamp(
                self._linear_speed + self._step, self._max_linear
            )
        elif name == "s":
            self._linear_speed = self._clamp(
                self._linear_speed - self._step, self._max_linear
            )
        elif name == "a":
            self._angular_speed = self._clamp(
                self._angular_speed + self._step, self._max_angular
            )
        elif name == "d":
            self._angular_speed = self._clamp(
                self._angular_speed - self._step, self._max_angular
            )
        else:
            return
        self._log_speeds()

    def _compute_twist(self):
        lin = self._linear_speed
        ang = self._angular_speed
        vx = (lin if "up" in self._held else 0.0) - (
            lin if "down" in self._held else 0.0
        )
        vy = (lin if "left" in self._held else 0.0) - (
            lin if "right" in self._held else 0.0
        )
        wz = (ang if "q" in self._held else 0.0) - (
            ang if "e" in self._held else 0.0
        )
        return round(vx, 4), round(vy, 4), round(wz, 4)

    def _is_moving(self):
        return any(key in self._held for key in MOTION_KEYS)

    # ------------------------------------------------------------------ 发布
    def _on_timer(self):
        now = time.monotonic()
        with self._lock:
            events = list(self._events)
            self._events.clear()
            for kind, name, stamp in events:
                if kind == "press":
                    self._on_press(name, stamp)
                else:
                    self._on_release(name, stamp)
            self._expire_holds(now)

            twist = self._compute_twist()
            # 按住运动键时持续重发；松开后（twist 变回 0）只补发一次
            publish = twist != self._last_twist or self._is_moving()
            if publish:
                self._last_twist = twist
            quit_requested = self._quit

        if publish:
            self._publish_twist(*twist)
        if quit_requested:
            self.publish_stop()
            self.get_logger().info("收到 Esc，退出")
            rclpy.shutdown()

    def _publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._publisher.publish(msg)
        self.get_logger().debug(
            "cmd_vel: vx=%.2f vy=%.2f wz=%.2f" % (vx, vy, wz)
        )

    def publish_stop(self):
        with self._lock:
            self._last_twist = (0.0, 0.0, 0.0)
        try:
            self._publish_twist(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001 - 退出阶段上下文可能已关闭
            pass


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        node.start_keyboard()
    except Exception as exc:  # noqa: BLE001 - 输入源不可用时给出明确提示
        node.get_logger().fatal("初始化键盘监听失败：%s" % exc)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_keyboard()
        node.publish_stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())