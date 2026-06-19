"""
用户自定义转向策略 — 热重载插件 (Python)
=========================================
编辑本文件并保存，仿真器会在 1 秒内自动重载（无需重启）。

接口说明
--------
必须实现 compute(driver, state) -> dict

driver 字段：
  throttle  float  [-1, +1]  负 = 制动/倒退，正 = 前进
  steering  float  [-1, +1]  +1 = 全左，-1 = 全右
  handbrake int    {0, 1}

state 字段（均为 Python list，下标 0=FL 1=FR 2=RL 3=RR）：
  t          float       仿真时间 [s]
  x, y, psi  float       位姿 [m, m, rad]
  vx, vy     float       车体系纵/横速度 [m/s]
  yaw_rate   float       横摆角速度 [rad/s]
  delta      list[float] 各轮实际转角 [rad]
  fz         list[float] 各轮垂直载荷 [N]
  torque_steer list[float] 各轮主销阻力矩 [N·m]
  steer_limit float      单轮最大转角 [rad]

返回值：
  {'delta_cmd': [fl, fr, rl, rr]}   单位 rad，超出 steer_limit 会被后端截断
"""

import math


def compute(driver: dict, state: dict) -> dict:
    """
    示例：理想阿克曼策略（Python 重实现）
    ICR 纵向坐标固定在后轴中心；由 steering 驱动横向 ICR。
    """
    steering = driver["steering"]
    steer_limit = state["steer_limit"]
    L = state.get("wheelbase", 3.16)
    tf = state.get("track_front", 1.72)
    tr = state.get("track_rear", 1.72)

    if abs(steering) < 1e-4:
        return {"delta_cmd": [0.0, 0.0, 0.0, 0.0]}

    # ICR 横向坐标（车体系，左正）
    icr_y = L / math.tan(steering * steer_limit)

    def wheel_delta(wheel_x: float, wheel_y: float) -> float:
        dx = wheel_x
        dy = icr_y - wheel_y
        return math.atan2(dx, dy)

    fl = wheel_delta(+L / 2, +tf / 2)
    fr = wheel_delta(+L / 2, -tf / 2)
    rl = wheel_delta(-L / 2, +tr / 2)
    rr = wheel_delta(-L / 2, -tr / 2)

    return {"delta_cmd": [fl, fr, rl, rr]}
