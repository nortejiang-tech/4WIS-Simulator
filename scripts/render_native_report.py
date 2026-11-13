#!/usr/bin/env python3
"""Render the measured native-core benchmark without inventing missing platforms."""

from pathlib import Path
import base64
import html
import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/reports/native_v0.106.0"


def main():
    inputs = sorted(OUT.glob("benchmark-*.json"))
    datasets = [
        (p.stem.removeprefix("benchmark-"), json.loads(p.read_text())) for p in inputs
    ]
    if not datasets:
        raise SystemExit("no measured benchmark JSON")
    lines = [
        "# C++ 仿真内核：性能与数值对照",
        "",
        "日期：2026-11-14。v0.106.0 独立预览分支；基线 v0.105.0。",
        "",
        "本报告比较保留的 Python 数值实现和 C++17 实现。控制器、故障调度、记录和接口保持同一执行链；没有改变外层 5 ms / 内部最高 1.25 ms 步长，也没有放宽原有物理或黄金回归标准。",
        "",
        "每个工况为 14 s、40 km/h 目标、1 s 起步斜坡、2°/0.2 Hz 正弦前轮输入。两类场景为双前轮同时卡滞，以及 FL 卡滞、RR 自由转向、FR 偏差和噪声、FL 掉线的错时组合。完整配置和事件时刻保存在 JSON。",
        "",
        "每种内核先预热一次，再交替顺序各运行 5 次，取完整 run_experiment（含记录）的墙钟耗时中位数。Mac 测量期间没有并行运行测试或打包。Mac 硬件实查为 Apple M5 Pro / 64 GiB。Windows 数据来自 GitHub 原生 runner（如列出）；云端机器的性能不等同于用户自己的 Windows 电脑。",
        "",
        "数值准则：逐通道 rtol=1e-9；状态通道 atol=1e-9 SI，力/力矩及摩擦余量 atol=1e-5。摩擦圆边缘的平方根余量会放大末位舍入差，初次 1e-8 统一绝对阈值在近零余量出现 7.49e-8 N 差异；因此新迁移对照按量纲设置容差。既有物理断言及黄金回归容差未改。时间、步数和故障时间线要求精确相同。",
        "",
    ]
    for platform, data in datasets:
        lines += [
            f"## {platform} 实测",
            "",
            f"状态：{data['status']}；Python {data['python'].split()[0]}；{data['platform']}。源提交 {data['source_commit']}；原生源指纹 {data['engine']['native_source_sha256']}。",
            "",
            "|模型|轮胎|故障组合|Python 中位 s|C++ 中位 s|提速|数值对照|",
            "|---|---|---|---:|---:|---:|---|",
        ]
        for c in data["cases"]:
            lines.append(
                f"|{c['model']}|{c['tire']}|{c['case']}|{c['median_seconds']['python']:.3f}|{c['median_seconds']['native']:.3f}|{c['speedup']:.2f}×|{c['parity']}|"
            )
        errors = [
            v
            for c in data["cases"]
            for k, v in c["max_abs_channel_differences"].items()
            if k in ("pose_x", "pose_y") and v is not None
        ]
        lines += [
            "",
            f"位置 x/y 分量的最大绝对差为 {max(errors):.3g} m。这是两份程序的数值一致性，不是实车位置精度。各次原始耗时、全部通道差值及实验配置见 benchmark-{platform}.json。",
            "",
        ]
    if not any(p == "windows" for p, _ in datasets):
        lines += ["Windows 测量：PENDING；尚未收到原生 runner 结果。", ""]
    lines += [
        "## 软件与物理验收边界",
        "",
        "Mac 完整门禁：后端 1119 项、前端 54 项、浏览器 E2E 58 项、6 个黄金工况、2 组解析基准/15 个指标通过。随后自定义轮胎回退与可变 K&C 缓存保护的定向验收 58 项通过。完整输出在 local-validation.txt / focused-validation.txt。包级与 Windows 最终状态以 acceptance.md 为准。",
        "",
        "新增检查覆盖两种模型/轮胎、同时/错时/恢复/观测故障、K&C、载荷敏感性、反向行驶、制动、详细转向机构，以及独立质心力矩平衡、轮速角冲量、摩擦约束、连杆虚功和内存缓冲区边界。原生数值失败直接暴露，不静默切换算法。",
        "",
        "实际收益主要针对批量仿真和 CPU 预算；实时控制频率与记录频率未提高。当前 C++ 内核不提供额外模型精度，也不改变原模型一阶分步积分、降阶悬架及传感器只影响观测等限制。台架/实车相关性仍为 PENDING。",
        "",
        "回退：设置 SIM4WIS_ENGINE=python 或使用保留的 v0.105.0 包。无用户数据格式迁移；本轮未执行数据回滚。正式 main/v0.105.0 未替换。",
        "",
    ]
    fig, axs = plt.subplots(
        len(datasets),
        1,
        figsize=(12, 4.4 * len(datasets)),
        squeeze=False,
        layout="constrained",
    )
    for ax, (platform, data) in zip(axs[:, 0], datasets):
        xs = np.arange(len(data["cases"]))
        w = 0.36
        ax.bar(
            xs - w / 2,
            [c["median_seconds"]["python"] for c in data["cases"]],
            w,
            label="Python reference",
            color="#6c8092",
        )
        ax.bar(
            xs + w / 2,
            [c["median_seconds"]["native"] for c in data["cases"]],
            w,
            label="C++ core",
            color="#1a8c81",
        )
        ax.set_xticks(
            xs,
            [
                f"{'Planar' if c['model'] == 'simplified_dynamic' else 'Multibody'}\n{c['tire']}\n{c['case']}"
                for c in data["cases"]
            ],
            fontsize=8,
        )
        ax.set(
            ylabel="Whole experiment median wall time [s]",
            title=f"{platform}: identical 14 s simulation; 5 alternating repetitions",
        )
        ax.grid(axis="y", alpha=0.2)
        ax.legend()
    fig.savefig(OUT / "performance.png", dpi=150)
    plt.close(fig)
    lines.insert(4, "![完整实验耗时](performance.png)")
    (OUT / "analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    blocks = []
    table = False
    for line in lines:
        if line.startswith("|"):
            if (
                set(line.replace("|", "").replace(":", "").replace("-", "").strip())
                == set()
            ):
                continue
            if not table:
                blocks.append("<table>")
                table = True
            blocks.append(
                "<tr>"
                + "".join(
                    "<td>" + html.escape(v) + "</td>"
                    for v in line.strip("|").split("|")
                )
                + "</tr>"
            )
            continue
        if table:
            blocks.append("</table>")
            table = False
        if line.startswith("!["):
            blocks.append(
                '<img alt="Measured runtime" src="data:image/png;base64,'
                + base64.b64encode((OUT / "performance.png").read_bytes()).decode()
                + '">'
            )
        elif line.startswith("# "):
            blocks.append("<h1>" + html.escape(line[2:]) + "</h1>")
        elif line.startswith("## "):
            blocks.append("<h2>" + html.escape(line[3:]) + "</h2>")
        elif line:
            blocks.append("<p>" + html.escape(line) + "</p>")
    (OUT / "analysis.html").write_text(
        '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>C++ 仿真内核实测</title><style>body{max-width:1120px;margin:auto;padding:36px;font:16px/1.8 system-ui,sans-serif;color:#17333d;background:#f4f7f8}h1,h2{color:#126c63}table{width:100%;border-collapse:collapse;background:white;font-size:13px}td{border:1px solid #d2dfe4;padding:7px}tr:first-child{font-weight:600;background:#e0eeec}img{width:100%}p{overflow-wrap:anywhere}</style>'
        + "".join(blocks)
        + "</html>",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
