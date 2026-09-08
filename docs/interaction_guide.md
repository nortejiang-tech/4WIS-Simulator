# 三种交互与结果输出使用说明

2026-09-09；当前开发分支。启动后的默认端口通常为 8010，本轮试用服务位于 `http://127.0.0.1:8023`。端口以实际启动命令为准。

## 人工驾驶

顶部选择「手动驾驶」。W/S 表达前进/后退意图，A/D 转向，Space 驻车制动；原有前进制动、重新按键后挂倒挡的逻辑保留。手柄和方向盘仍使用驾驶面板的 Gamepad 映射、死区、方向反转和校准设置；设备须由浏览器 Gamepad API 识别。

输入仅在驾驶页获得焦点时有效。编辑文本、切换页面、窗口失焦、网页隐藏或连接断开都会释放人工输入；服务端另有约 0.75 s 输入超时释放。断连期间的驾驶命令不会排队重放。

「暂停仿真」冻结积分时钟；「停止输入」释放驾驶输入并停止实时脚本，车辆按物理模型继续运动，并不瞬间归零车速。之后点击「启用手动输入」才能重新接收手动驾驶命令。

## 脚本工况

顶部「脚本工况」提供车辆视图、YAML 编辑/工况库、启动/停止、暂停和录制。脚本运行期间拥有驾驶台输入；脚本结束、出错或停止后释放输入，错误显示在面板中。

```yaml
script:
  name: steer_then_stop
  loop: false
  actions:
    - {t: 0, action: set_strategy, name: ideal_ackermann}
    - {t: 0, action: drive, throttle: 0.1, steering: 0}
    - {t: 2, action: steer_ramp, from_: 0, to: 0.05, duration: 0.5}
    - {t: 5, action: brake, duration: 2}
    - {t: 7, action: stop}
```

`t` 是本轮脚本开始后的仿真秒，排序后依次执行。渐变动作阻塞到持续时间完成，因此后续动作不会在它中途抢占。`wait_until` 可等距离 `distance`、速度阈值 `speed_below` 或绝对仿真时间 `until_t`；没有额外谓词时只等待动作的 `t`。暂停仿真也暂停动作计时。重置会保留本轮脚本已流逝时间的连续性。

实时脚本按约 50 Hz 轮询仿真时钟，动作落点存在调度粒度。需要严格重复、参数扫描或长研究时使用「离线批量试验」的 Experiment/study，或以下整步 Agent API。

## Agent 接口

首先请求 `GET /api/agent/capabilities`，取得 `4wis.agent.v1` 契约、完整创建/控制 JSON Schema、通道/单位、支持策略和限制。`/docs` 是交互式 OpenAPI，`/openapi.json` 是机器可读文档。

| 操作 | HTTP | MCP 工具 |
|---|---|---|
| 发现能力 | `GET /api/agent/capabilities` | `describe_capabilities` |
| 创建独立会话 | `POST /api/agent/sessions` | `create_session` |
| 列出会话 | `GET /api/agent/sessions` | `list_sessions` |
| 观察状态 | `GET /api/agent/sessions/{id}` | `observe_session` |
| 保持控制并推进 | `POST /api/agent/sessions/{id}/step` | `step_session` |
| 抽样图形数据 | `GET /api/agent/sessions/{id}/preview` | 使用 REST；MCP 观察回执含最新遥测 |
| 保存持久结果 | `POST /api/agent/sessions/{id}/export` | `export_session` |
| 释放内存会话 | `DELETE /api/agent/sessions/{id}` | `close_session` |
| 获取结果清单 | `GET /api/runs/{run_id}/artifacts` | `get_run_artifacts` |
| 异步研究 | `POST /api/study/run`、`GET /api/study/jobs/{job_id}` | `start_study`、`get_study_job` |

创建示例，建议明确指定模型、策略、步长，避免依赖默认值：

```json
{
  "label": "Agent 闭环实验",
  "experiment": {
    "name": "agent_example",
    "model_type": "simplified_dynamic",
    "strategy": "ideal_ackermann",
    "dt": 0.005
  }
}
```

返回 `session_id`、`revision=0`、状态、限制和样本数。会话与人工驾驶台隔离。车辆初始参数来自显式指定 profile/overrides 或模型默认值，不隐式复制人工窗口中的未保存参数。

```json
{
  "request_id": "step-001",
  "expected_revision": 0,
  "steps": 200,
  "control": {"throttle": 0.1, "steering": 0.05, "brake": 0, "gear": 1, "handbrake": 0}
}
```

上述请求保持控制值推进 200 个 5 ms 积分步，共 1 s 仿真时间。下一次用返回的修订号。每次提交完整控制，省略字段回到 Schema 默认值；静态策略设置使用 `experiment.mode_params`，动态控制值只能放 `step.control`，避免隐藏的持续输入。

- `throttle/brake` 范围 `[0,1]`，`steering` 为 `[-1,1]`；`gear=-1/0/1` 表示 R/N/D，`handbrake=0/1`。踏板语义仍由车辆的 `longitudinal_mode` 决定；默认速度伺服与扭矩驱动不是同一种动力总成。
- `speed_target_ms` 是显式速度参考，`front_angle_rad` 是显式前轮角参考，绕过相应驾驶映射；`front_angle_rad` 仅适用于 ideal_ackermann/rear_wheel_steer/fault_reconfig。manual_body/zero_radius 不支持纵向速度参考。
- `manual_wheel` 使用 `wheel_norm=[FL,FR,RL,RR]`；`manual_body` 使用 `body_fraction=[vx,vy,yaw]`。每项为 `[-1,1]`。不兼容的专用控制返回 422。
- 单次 1–1,000 步，每会话最多 20,000 步，最多 4 个活动会话。闲置回收阈值 1 h，在创建会话时清理；会话位于内存，重启会丢失。需要持续存档必须 export。
- 响应不确定时，用**原 request_id 和完全相同请求体**重试。返回 `replayed=true` 不再次积分。更改原请求体、过期 revision 返回 409；非法输入 422；未知会话 404；会话数已满 429。
- 数值/执行异常进入 `failed`，保留已接受步的部分数据及错误；后续控制拒绝，仍可导出。失败回执是 422，不会自动 reset 后伪装成功。
- 导出的同一修订只生成一个 run；后续推进后再次导出产生新 run，之前的数据不改写。关闭会话不会删除已导出 run。

坐标使用车体 X 向前、Y 向左；位置 m、速度 m/s、角度 rad、力 N、力矩 N·m。轮序始终 FL/FR/RL/RR。每轮和附着相关信号须结合 `grip_valid`、`steer_plant_active` 等有效性信息解释；运动学层不提供真实轮胎力预测。逐通道单位以能力清单和 run manifest 为准。

MCP 已有 8 个工具保持兼容，新增 9 个，共 17 个。开发态宿主可配置如下本地进程；这里仅给出配置，没有修改任何宿主：

```json
{
  "command": "/Users/nortepro/Dev/4WIS_Simulator/4WIS Simulator/backend/.venv/bin/python",
  "args": ["-m", "sim4wis_mcp.server", "--base", "http://127.0.0.1:8023"]
}
```

上述模块入口已用当前虚拟环境验证可导入，不依赖尚未生成的 `sim4wis-mcp` console script。API 返回相对工件 URL，MCP 转为绝对 URL。推荐 Agent 先看状态/汇总，按需下载全量结果，避免把大数组直接塞入上下文。

### 便携版 MCP 配置

`v0.104.0` 的 macOS 与 Windows 便携包均包含 `agent_mcp` 启动器、`sim4wis_mcp` 源码、MCP Python SDK 和
Agent 的 golden 验证支持文件。解压后不要手抄路径：在该目录的终端执行以下命令，让包输出包含当前绝对路径的
JSON 配置，再复制到宿主。

```sh
# macOS
./agent_mcp.command --print-config

# Windows cmd.exe
agent_mcp.bat --print-config
```

macOS 的输出以 `agent_mcp.command` 为 `command`。Windows 的输出使用
`cmd.exe /d /c <解压目录>\\agent_mcp.bat`，保证带空格的解压路径与 `.bat` 调用都正确。MCP 进程默认连
`http://127.0.0.1:8010`；未发现后端时只启动自己创建的本地后端，并在退出时只终止该子进程。需要改端口时，在宿主
配置中为该进程设置 `SIM4WIS_BACKEND_HTTP`。

## 图形、全量数据和图表

人工/脚本勾选「全部可用通道 · 每个积分步采样」，开始录制，停止后可导出 CSV 或「保存到结果库」。若确认不要这一段数据，点击「丢弃录制」清空停止后的缓冲；该动作不会重置车辆或改变已保存的 run。Agent 每个已接受积分步自动采样，点击「保存完整结果」。两条路径使用相同的 158 个遥测通道；CSV 另含时间，保存实时记录时另有策略索引及元数据中的策略字典。旧批跑/历史 run 保留其原通道和采样频率。

| 输出 | 内容与精度 |
|---|---|
| 工作台图形 | 人工/脚本的 2D/3D；Agent 的独立轨迹、轮位读数和横摆曲线 |
| CSV | 所有已记录行，float64 使用 17 位有效数字往返；不再用 6 位格式损失精度 |
| JSON | 同一 CSV 的全部数值行与 manifest；缺失/非有限数值为 null，CSV 对应空字段 |
| ZIP | 原始 data.csv、meta.json、manifest.json、trajectory.svg、chart.svg；图形不可用时保留说明 |
| SVG / 分析页 | 等比例轨迹及时间曲线、现有叠图/KPI/PNG/回放功能；预览明确标注抽样 |

manifest 给出样本数、通道单位、采样方式、文件大小和 CSV SHA-256。Agent 预览默认至多 300 点、上限 1,000 点；导出 SVG 至多约 2,000 点。这些用于查看，不能代替全量峰值分析。

实时缓冲默认保留约 30 min。溢出明确显示丢弃样本数，`complete=false`；重置时间轴时自动停止录制并标记不完整。全量表示当前 run 的全部已记录样本，不能恢复被缓冲淘汰的数据。长时实验建议使用有明确终点的批跑/study，目前未实现无限期磁盘流式录制。

实际键盘与浏览器流程、真实 HTTP 和 stdio MCP 已验收；物理手柄/方向盘的设备识别、轴极性、踏板量程及力反馈需要接入真实设备后单独验收。本轮没有增加或验证力反馈输出。模型与实车/台架的外部相关性仍待验证。
