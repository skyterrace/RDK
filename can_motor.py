#!/usr/bin/env python3

import can
import time
import argparse
import struct

def parse_args():
    parser = argparse.ArgumentParser(
        description="DM-S2325 CAN control"
    )
    parser.add_argument(
        "--channel",
        default="can0"
    )
    parser.add_argument(
        "--motor",
        type=int,
        default=1,
        help="motor id 1-7"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0,
        help="-45.0~45.0rad/s"
    )
    return parser.parse_args()

def speed_to_bytes(speed_rad_s: float) -> list:
    """
    将速度（rad/s）转换为低位在前、高位在后的4字节列表。
    取值范围：-45.0 ~ 45.0
    """
    if not -45.0 <= speed_rad_s <= 45.0:
        raise ValueError(f"速度 {speed_rad_s} 超出范围 [-45, 45] rad/s")

    # '<f' 表示小端序(little-endian)的单精度浮点数
    data = struct.pack('<f', speed_rad_s)
    return list(data)

# ===== 量程配置 =====
P_MAX = 12.5      # rad，位置满量程 ±12.5
V_MAX = 200.0      # rad/s，速度满量程 ±90
T_MAX = 10.0   # Nm，扭矩满量程 ±18


def parse_can_feedback(data) -> dict:
    """
    解析 8 字节 CAN 反馈报文。

    字节定义：
        D[0] = ID | ERR<<4
        D[1] = POS[15:8]
        D[2] = POS[7:0]
        D[3] = VEL[11:4]
        D[4] = VEL[3:0] | T[11:8]
        D[5] = T[7:0]
        D[6] = T_MOS   (℃)
        D[7] = T_Rotor (℃)

    位置：16 位有符号，线性映射 ±12.5 rad
    速度：12 位偏移编码（0 对应 2048），线性映射 ±90 rad/s
    扭矩：12 位偏移编码（0 对应 2048），线性映射 ±18 Nm
    """
    if len(data) < 8:
        raise ValueError("报文长度不足 8 字节")
    d = list(data[:8])

    # ---- D[0]: ID | ERR<<4 ----
    ctrl_id = d[0] & 0x0F
    err_code = (d[0] >> 4) & 0x0F
    ERR_MAP = {
        0x0: "失能", 0x1: "使能", 0x5: "读出传感器错误",
        0x6: "读取电机参数错误", 0x8: "超压", 0x9: "欠压",
        0xA: "过电流", 0xB: "MOS过温", 0xC: "电机线圈过温",
        0xD: "通讯丢失", 0xE: "过载",
    }
    err_str = ERR_MAP.get(err_code, f"未知状态(0x{err_code:X})")

    # ---- POS: D[1] 高 8 位，D[2] 低 8 位，16 位有符号 ----
    pos_raw = (d[1] << 8) | d[2]
    if pos_raw >= 0x8000:
        pos_raw -= 0x10000
    position = pos_raw * P_MAX / 32767.0          # rad

    # ---- VEL: D[3] 高 8 位，D[4] 低 4 位，12 位偏移编码 ----
    vel_u = (d[3]<<4) | ((d[4] >> 4) & 0x0F)
    vel_u &= 0x0FFF
    vel_raw = vel_u
    velocity = vel_u*2.0*V_MAX/4095.0 - V_MAX          # rad/s

    # ---- T: D[4] 低 4 位 为高 4 位，D[5] 为低 8 位，12 位偏移编码 ----
    t_u = ((d[4] & 0x0F)<<8) | (d[5]) & 0x0FFF
    t_raw = t_u
    torque = t_u*2.0*T_MAX/4095.0 - T_MAX           # Nm

    return {
        "controller_id": ctrl_id,
        "error_code": err_code,
        "error_str": err_str,
        "pos_raw": pos_raw,
        "position": position,   # rad
        "vel_raw": vel_raw,
        "velocity": velocity,   # rad/s
        "torque_raw": t_raw,
        "torque": torque,       # Nm
        "t_mos": d[6],          # ℃
        "t_rotor": d[7],        # ℃
    }

class DM2325:

    def __init__(self, channel, motor_id):
        self.bus = can.interface.Bus(
            channel=channel,
            interface="socketcan"
        )
        self.id = motor_id

    def enable_motor(self):
        data=[0xFF]*8
        data[7]=0xFC
        msg=can.Message(
            arbitration_id=0x200+self.id,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)

    def disable_motor(self):
        data=[0xFF]*8
        data[7]=0xFD
        msg=can.Message(
            arbitration_id=0x200+self.id,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)

    def send_speed(self, value):
        value = max(
            -45,
            min(45,value)
        )
        #data=[0]*4
        #data[3] = 0xBF
        data=speed_to_bytes(value)
        msg=can.Message(
            arbitration_id=0x200+self.id,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)

    def read_feedback(self):
        while True:
            msg=self.bus.recv(
                timeout=0.1
            )
            if msg is None:
                continue
            if msg.arbitration_id == (
                0x00
            ):
                data=msg.data
                return parse_can_feedback(data)

    def close(self):
        if self.bus is not None:
            self.bus.shutdown()
            print("CAN bus closed")
            
def main():
    args=parse_args()
    motor=DM2325(
        args.channel,
        args.motor
    )
    print(
        "DM2325 start"
    )
    motor.enable_motor()
    try:
        while True:
            # 控制
            motor.send_speed(
                args.speed
            )
            #读取
            state=motor.read_feedback()
            for k,v in state.items():
                print(f"{k:>15}:{v}")
            time.sleep(
                1.02
            )
    except KeyboardInterrupt:
        print("Exit by user")
    finally:
        motor.disable_motor()
        motor.close()

if __name__=="__main__":

    main()
