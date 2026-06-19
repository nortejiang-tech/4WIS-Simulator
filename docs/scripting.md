# 动作脚本 DSL — Phase 2

声明式 YAML 动作序列，用于：
- 重复跑标准工况（slalom、双移线、零半径演示等）
- 自动化测试 / 回归基准
- 演示和教学

脚本只能调用预定义动作 —— 不执行任意 Python，避免安全风险。
复杂控制策略请通过 Simulink FMU 接入（见 `fmu_integration.md`）。

## 顶层结构

```yaml
script:
  name: my_script              # 显示在前端
  description: ""
  loop: false                  # 是否循环
  actions:
    - { t: 0.0, action: ..., ... }
    - { t: 1.5, action: ..., ... }
```

`actions` 列表会按 `t` 排序后顺序执行。每个动作的 `t` 是从脚本启动算起的绝对秒数。

## 动作清单

### `drive` —— 设定瞬时油门 / 转向

```yaml
- { t: 0.0, action: drive, throttle: 0.5, steering: 0.0 }
```

### `set_strategy` —— 切换控制策略

```yaml
- { t: 4.5, action: set_strategy, name: crab }
```

`name` 必须是 `/api/strategies` 返回的某个名字（含内置 + FMU 插件）。

### `set_mode_params` —— 调整当前策略的 mode_params

```yaml
- { t: 1.0, action: set_mode_params, params: { rear_ratio: -0.5 } }
```

### `throttle_ramp` / `steer_ramp` —— 线性渐变

```yaml
- { t: 2.0, action: steer_ramp, from_: 0.0, to: +0.4, duration: 0.4 }
- { t: 5.0, action: throttle_ramp, from_: 0.5, to: 0.0, duration: 1.0 }
```

注意：参数名是 `from_`（带下划线），不是 Python 关键字 `from`。

### `brake` —— 持续制动一段时间，或直到车辆停下

```yaml
- { t: 8.0, action: brake, duration: 2.0 }
```

会自动判断 `|vx| < 0.05 m/s` 后提前结束。

### `wait_until` —— 阻塞到某条件满足

```yaml
- { t: 4.0, action: wait_until, speed_below: 0.1 }
- { t: 4.0, action: wait_until, distance: 50.0 }
- { t: 4.0, action: wait_until, t: 30.0 }    # 直到仿真时间 t >= 30
```

可以指定 `t`、`distance`（从脚本起点算起的位置距离）、`speed_below`（速度门限）三者之一。

### `reset` —— 重置车辆位姿与轨迹

```yaml
- { t: 0.0, action: reset }
```

### `stop` —— 终止脚本

```yaml
- { t: 15.0, action: stop }
```

`loop: true` 时 `stop` 同时会跳出循环。

---

## 内置脚本库

`<repo>/scripts_lib/` 里有几个常用样例：

| 文件 | 工况 |
|------|------|
| `slalom.yaml` | 3 m 间距小幅 slalom |
| `double_lane_change.yaml` | 简化 DLC 工况 |
| `crab_park.yaml` | 直行后切蟹行入位 |
| `spin_demo.yaml` | 零半径转向演示 |

前端 `ScriptPanel` 的下拉里直接选 → "载入" → "启动"。

## 程序化执行

```python
from sim4wis.input.action_schema import Script
from sim4wis.input.script import ScriptRunner

script = Script.from_dict(yaml.safe_load(open("scripts_lib/slalom.yaml")))
runner = ScriptRunner(sim)
runner.load(script)
await runner.start()
```

## 行为细节

- 脚本运行期间会覆盖键盘输入；脚本停止后自动恢复键盘控制
- 脚本结束不会重置位姿（需要 explicit `reset` 动作）
- 多个 ramp 重叠会按它们 t 的顺序执行（后者覆盖前者）
- 异常 / 未知动作只记录 warning 日志，不中断脚本

## 调试

启动脚本后，`/api/script/status` 返回：
```json
{
  "running": true,
  "current_action_idx": 3,
  "t_in_script": 2.45,
  "script_name": "double_lane_change",
  "loop_count": 0
}
```

前端 ScriptPanel 每 500 ms 轮询一次状态，显示当前动作索引。
