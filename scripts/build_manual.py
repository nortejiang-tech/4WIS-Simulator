#!/usr/bin/env python3
"""生成图文版《使用说明书》— 自动截图 + 自包含 HTML。

    python scripts/build_manual.py               # 全流程：起后端→截图→出 HTML
    python scripts/build_manual.py --skip-capture  # 只用已有截图重排 HTML

原理：起一个临时后端（8016，托管已构建的前端 dist），用 playwright 无头
Chromium 把每个页面/面板真实跑一遍（含手柄各模式的效果演示——经 WS 注入
mode_params 复现摇杆输出），截图存 docs/manual_figs/，再生成
docs/user_manual.html（图片 base64 内嵌，单文件可分发）。

发版时把 user_manual.html 拷为 4WIS_Simulator_v{ver}_使用说明书.html。
"""

from __future__ import annotations

import argparse
import base64
import io
import math
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend" / "src"))

from sim4wis import __version__ as VER  # noqa: E402

PORT = 8016
BASE = f"http://127.0.0.1:{PORT}"
FIG_DIR = ROOT / "docs" / "manual_figs"
OUT_HTML = ROOT / "docs" / "user_manual.html"


def frames_to_gif(frames_png: list[bytes], path: Path,
                  width: int = 560, fps: int = 10, colors: int = 96) -> None:
    """Assemble PNG frame bytes into a looping GIF with one shared palette
    (consistent palette across frames → no inter-frame flicker, smaller file)."""
    from PIL import Image

    imgs: list = []
    for b in frames_png:
        im = Image.open(io.BytesIO(b)).convert("RGB")
        if im.width > width:
            h = round(im.height * width / im.width)
            im = im.resize((width, h), Image.LANCZOS)
        imgs.append(im)
    base = imgs[len(imgs) // 2].quantize(colors=colors, method=Image.MEDIANCUT)
    pframes = [im.quantize(palette=base, dither=Image.NONE) for im in imgs]
    pframes[0].save(path, save_all=True, append_images=pframes[1:],
                    duration=round(1000 / fps), loop=0, optimize=True, disposal=2)


# ─────────────────────────────────────────────────────────────────────────────
# 截图巡游
# ─────────────────────────────────────────────────────────────────────────────

def wait_health(timeout: float = 25.0) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with urllib.request.urlopen(f"{BASE}/health", timeout=1) as r:
                if r.status == 200:
                    return
        except Exception:
            time.sleep(0.3)
    raise RuntimeError("backend did not become healthy")


def capture() -> dict[str, bool]:
    from playwright.sync_api import sync_playwright

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    ok: dict[str, bool] = {}

    env = dict(**__import__("os").environ)
    env["PYTHONPATH"] = str(ROOT / "backend" / "src")
    backend = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "sim4wis.main:app",
         "--host", "127.0.0.1", "--port", str(PORT), "--log-level", "warning"],
        cwd=str(ROOT / "backend"), env=env,
    )
    try:
        wait_health()
        with sync_playwright() as pw:
            browser = pw.chromium.launch()
            page = browser.new_page(viewport={"width": 1600, "height": 1000},
                                    device_scale_factor=1.5)
            page.goto(BASE, wait_until="networkidle")
            page.wait_for_timeout(1200)

            def shot(name: str, locator=None, full=False) -> None:
                try:
                    path = str(FIG_DIR / f"{name}.png")
                    if locator is not None:
                        locator.screenshot(path=path)
                    else:
                        page.screenshot(path=path, full_page=full)
                    ok[name] = True
                    print(f"  ✓ {name}")
                except Exception as e:  # noqa: BLE001
                    ok[name] = False
                    print(f"  ✗ {name}: {e}")

            def rail(label: str, settle: int = 900) -> None:
                page.locator(".rail-item", has_text=label).click()
                page.wait_for_timeout(settle)

            def panel(title: str):
                return page.locator(".panel", has=page.locator(".panel-name", has_text=title))

            def ws_driver(js_obj: str) -> None:
                """在页面里经 WS 注入一条 driver 消息（复现手柄输出）。"""
                page.evaluate(
                    """async (payload) => {
                        if (!window.__mws || window.__mws.readyState !== 1) {
                            window.__mws = new WebSocket(`ws://${location.host}/ws/state`);
                            await new Promise(r => window.__mws.onopen = r);
                        }
                        window.__mws.send(JSON.stringify(Object.assign({type:'driver'}, payload)));
                    }""",
                    __import__("json").loads(js_obj),
                )

            def reset() -> None:
                page.evaluate("fetch('/api/reset', {method:'POST'})")
                page.wait_for_timeout(400)

            viewport_pane = page.locator(".viewport-pane")

            # 00 快速开始
            shot("00_quickstart")
            page.locator("button[aria-label='关闭快速开始']").click()
            page.wait_for_timeout(300)

            # 01 驾驶一段（键盘 WASD 真实驱动）→ 全页
            page.keyboard.down("w")
            page.wait_for_timeout(2400)
            page.keyboard.down("a")
            page.wait_for_timeout(1600)
            page.keyboard.up("a")
            page.keyboard.up("w")
            page.wait_for_timeout(700)
            shot("01_run_overview")

            # 02 3D 视图
            page.locator(".viewport-pane button", has_text="3D").first.click()
            page.wait_for_timeout(1300)
            shot("02_view3d", viewport_pane)
            page.locator(".viewport-pane button", has_text="2D").first.click()
            page.wait_for_timeout(500)

            # 03/04/05 面板特写
            shot("03_model_panel", panel("动力学模型"))
            shot("04_strategy_panel", panel("控制策略"))
            shot("05_drive_panel", panel("驾驶输入"))

            # 06 蟹行演示
            reset()
            page.evaluate("fetch('/api/strategy',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:'crab'})})")
            page.wait_for_timeout(400)
            ws_driver('{"throttle":0.25,"steering":0.5}')
            page.wait_for_timeout(2800)
            ws_driver('{"throttle":0,"steering":0.5}')
            page.wait_for_timeout(400)
            shot("06_crab_drive", viewport_pane)
            ws_driver('{"throttle":0,"steering":0}')

            # 07 手柄面板（经典）
            gp_panel = panel("手柄映射与校准")
            gp_panel.scroll_into_view_if_needed()
            page.wait_for_timeout(300)
            shot("07_gp_panel", gp_panel)

            # 手柄直控演示：放大画布看车轮
            reset()
            for _ in range(5):
                page.locator(".viewport-pane button", has_text="+").first.click()
            page.wait_for_timeout(400)

            def gp_demo(preset: str, payload: str, fig: str, settle: int = 1200) -> None:
                page.locator(".gp-preset", has_text=preset).click()
                page.wait_for_timeout(500)
                ws_driver(payload)
                page.wait_for_timeout(settle)
                shot(fig, viewport_pane)

            # 08 前后轴独立：前 +0.5 / 后 −0.5
            gp_demo("前后轴独立",
                    '{"throttle":0,"steering":0,"mode_params":{"wheel_norm":[0.5,0.5,-0.5,-0.5]}}',
                    "08_gp_frontrear")
            # 09 逐轮直控：四轮各不相同
            gp_demo("逐轮直控",
                    '{"throttle":0,"steering":0,"mode_params":{"wheel_norm":[0.5,-0.25,0.15,-0.55]}}',
                    "09_gp_perwheel")
            # 10 全向：斜移 + 自转轨迹
            for _ in range(5):
                page.locator(".viewport-pane button", has_text="−").first.click()
            reset()
            page.locator(".gp-preset", has_text="全向车身").click()
            page.wait_for_timeout(500)
            ws_driver('{"throttle":0,"steering":0,"mode_params":{"vx_frac":0.25,"vy_frac":0.25,"yaw_frac":0.18}}')
            page.wait_for_timeout(4200)
            ws_driver('{"throttle":0,"steering":0,"mode_params":{"vx_frac":0,"vy_frac":0,"yaw_frac":0}}')
            page.wait_for_timeout(300)
            shot("10_gp_holonomic", viewport_pane)
            # 回到经典（恢复 ideal_ackermann）
            page.locator(".gp-preset", has_text="经典").click()
            page.wait_for_timeout(400)
            reset()

            # 11 场景页：小镇 + 双移线路径
            rail("场景")
            page.locator("button", has_text="小镇").first.click()
            page.wait_for_timeout(1600)
            page.evaluate(
                """() => {
                    const panels = Array.from(document.querySelectorAll('.panel'));
                    const p = panels.find(x => x.textContent.includes('轨迹'));
                    const sel = p?.querySelector('select');
                    if (!sel) return 'no select';
                    const opt = Array.from(sel.options).find(o => o.textContent.includes('双移线'));
                    if (!opt) return 'no option';
                    const setter = Object.getOwnPropertyDescriptor(HTMLSelectElement.prototype, 'value').set;
                    setter.call(sel, opt.value);
                    sel.dispatchEvent(new Event('change', {bubbles: true}));
                    return 'set';
                }"""
            )
            page.wait_for_timeout(300)
            page.locator("button", has_text="生成").first.click()
            page.wait_for_timeout(1000)
            shot("11_scene_page")

            # 12 扰动（API 放置冰面 + 减速带）
            page.evaluate(
                """async () => {
                    const mk = (b) => fetch('/api/scene/disturbances', {method:'POST',
                        headers:{'Content-Type':'application/json'}, body: JSON.stringify(b)});
                    await mk({type:'ice_patch', x: 30, y: 2, width: 8, length: 14, mu: 0.15});
                    await mk({type:'speed_bump', x: 15, y: 0, width: 8, length: 0.6, height: 0.05, stiffness: 80000});
                }"""
            )
            page.wait_for_timeout(900)
            shot("12_disturbance", viewport_pane)
            shot("13_fault_panel", panel("故障注入"))
            page.evaluate("fetch('/api/scene/clear',{method:'POST'})")

            # 14 车辆页
            rail("车辆")
            shot("14_vehicle_page")

            # 15 试验页：载入示例 → 2 策略矩阵 → 跑完出 KPI
            rail("试验")
            page.locator(".wf-list-item", has_text="iso3888_dlc_60kmh").click()
            page.wait_for_timeout(600)
            for s in ("ideal_ackermann", "rear_wheel_steer"):
                page.locator(".wf-chip", has_text=s).first.click()
            page.wait_for_timeout(300)
            page.locator("button.wf-btn.primary.big", has_text="运行").click()
            for _ in range(60):
                page.wait_for_timeout(500)
                if page.locator(".wf-job", has_text="完成").count() > 0:
                    break
            page.wait_for_timeout(400)
            shot("15_experiment_page")

            # 16/17/18 分析页
            rail("分析", settle=1200)
            items = page.locator(".wf-list-item")
            for i in range(min(4, items.count())):
                items.nth(i).click()
                page.wait_for_timeout(250)
            page.wait_for_timeout(1800)
            shot("16_analysis_kpi")
            page.evaluate("document.querySelector('.wf-main').scrollTop = 1e6")
            page.wait_for_timeout(600)
            shot("17_analysis_charts")
            page.evaluate("document.querySelector('.wf-main').scrollTop = 0")
            page.locator("button", has_text="▶ 回放").click()
            page.wait_for_timeout(2200)
            page.evaluate(
                """() => {
                    const s = document.querySelector('.replay-controls input[type=range]');
                    if (!s) return;
                    const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
                    set.call(s, String(Number(s.max) * 0.55));
                    s.dispatchEvent(new Event('input', {bubbles: true}));
                }"""
            )
            page.wait_for_timeout(800)
            shot("18_replay")

            # 19 负载特性
            rail("负载", settle=1500)
            for label in ("计算", "扫描", "运行"):
                btn = page.locator("button", has_text=label)
                if btn.count() > 0:
                    try:
                        btn.first.click(timeout=1500)
                        break
                    except Exception:
                        pass
            page.wait_for_timeout(3500)
            shot("19_load_page")

            # 20 原理简介
            rail("原理", settle=1500)
            shot("20_theory_page")

            # 21 ⌘K 命令面板
            page.keyboard.press("Control+k")
            page.wait_for_timeout(400)
            page.keyboard.type("试验")
            page.wait_for_timeout(400)
            shot("21_cmdk")
            page.keyboard.press("Escape")

            # ── 动图（GIF）：手柄各模式 + 驾驶，真跑连拍合成 ────────────
            rail("运行", settle=900)

            def set_strat(name: str) -> None:
                page.evaluate(
                    "(n)=>fetch('/api/strategy',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:n})})",
                    name)
                page.wait_for_timeout(300)

            def zoom(n: int) -> None:
                sym = "+" if n > 0 else "−"
                for _ in range(abs(n)):
                    page.locator(".viewport-pane button", has_text=sym).first.click()
                page.wait_for_timeout(200)

            def grab_gif(name: str, setup, step, n: int = 26, interval: int = 100,
                         zoom_in: int = 0, fps: int = 10) -> None:
                setup()
                if zoom_in:
                    zoom(zoom_in)
                page.wait_for_timeout(250)
                frames = []
                for i in range(n):
                    step(i)
                    page.wait_for_timeout(interval)
                    frames.append(viewport_pane.screenshot(type="png"))
                gp = FIG_DIR / f"{name}.gif"
                frames_to_gif(frames, gp, fps=fps)
                ok[name] = True
                print(f"  ✓ {name}.gif ({gp.stat().st_size/1e6:.1f} MB)")
                if zoom_in:
                    zoom(-zoom_in)

            def wn(vals) -> str:
                return '{"throttle":0,"steering":0,"mode_params":{"wheel_norm":[%f,%f,%f,%f]}}' % tuple(vals)

            # 1) 前后轴独立：前后轴反相扫角（车静止，看轮子独立摆）
            grab_gif("gif_frontrear",
                     lambda: (reset(), set_strat("manual_wheel")),
                     lambda i: ws_driver(wn([(a := 0.6 * math.sin(2 * math.pi * i / 26)), a,
                                             -0.6 * math.sin(2 * math.pi * i / 26),
                                             -0.6 * math.sin(2 * math.pi * i / 26)])),
                     zoom_in=3)

            # 2) 逐轮直控：四轮各自相位，完全独立摆
            grab_gif("gif_perwheel",
                     lambda: (reset(), set_strat("manual_wheel")),
                     lambda i: ws_driver(wn([0.55 * math.sin(2 * math.pi * i / 26 + ph)
                                             for ph in (0, 1.57, 3.14, 4.71)])),
                     zoom_in=3)

            # 3) 蟹行：整车横移
            grab_gif("gif_crab",
                     lambda: (reset(), set_strat("crab")),
                     lambda i: ws_driver('{"throttle":0.2,"steering":0.5}'),
                     n=30, interval=100)

            # 4) 全向车身：斜移 + 自转
            grab_gif("gif_holonomic",
                     lambda: (reset(), set_strat("manual_body")),
                     lambda i: ws_driver('{"throttle":0,"steering":0,"mode_params":{"vx_frac":0.2,"vy_frac":0.2,"yaw_frac":0.16}}'),
                     n=32, interval=100)

            # 5) 驾驶：绕桩（正弦转向 + 油门）
            grab_gif("gif_drive",
                     lambda: (reset(), set_strat("ideal_ackermann"), ws_driver('{"throttle":0,"steering":0}')),
                     lambda i: ws_driver('{"throttle":0.4,"steering":%f}' % (0.7 * math.sin(2 * math.pi * i / 16))),
                     n=32, interval=100)

            ws_driver('{"throttle":0,"steering":0}')
            reset()

            browser.close()
    finally:
        backend.terminate()
        try:
            backend.wait(timeout=5)
        except Exception:
            backend.kill()
    return ok


# ─────────────────────────────────────────────────────────────────────────────
# HTML 组装
# ─────────────────────────────────────────────────────────────────────────────

def _img(name: str, caption: str = "") -> str:
    p = FIG_DIR / f"{name}.png"
    if not p.exists():
        return f"<p class='miss'>（截图 {name} 缺失——重跑 build_manual.py）</p>"
    b64 = base64.b64encode(p.read_bytes()).decode()
    cap = f"<figcaption>{caption}</figcaption>" if caption else ""
    return f"<figure><img src='data:image/png;base64,{b64}' loading='lazy'/>{cap}</figure>"


def _gif(name: str, caption: str = "") -> str:
    p = FIG_DIR / f"{name}.gif"
    if not p.exists():
        return f"<p class='miss'>（动图 {name} 缺失——重跑 build_manual.py）</p>"
    b64 = base64.b64encode(p.read_bytes()).decode()
    cap = f"<figcaption>▶ {caption}（动图）</figcaption>" if caption else ""
    return f"<figure class='gif'><img src='data:image/gif;base64,{b64}' loading='lazy'/>{cap}</figure>"


def build_html() -> None:
    img = _img
    gif = _gif
    html = f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>4WIS Simulator v{VER} 使用说明书（图文版）</title>
<style>
 body {{ font-family:"PingFang SC","Hiragino Sans GB","Microsoft YaHei",sans-serif;
        max-width: 1020px; margin: 0 auto; padding: 36px 22px; color:#1a202c; line-height:1.8; }}
 h1 {{ font-size: 26px; border-bottom: 3px solid #1f77b4; padding-bottom: 10px; }}
 h2 {{ font-size: 20px; margin-top: 2.4em; border-left: 5px solid #1f77b4; padding-left: 10px; }}
 h3 {{ font-size: 16px; margin-top: 1.6em; }}
 figure {{ margin: 14px 0 22px; text-align: center; }}
 figure img {{ max-width: 100%; border: 1px solid #cbd5e0; border-radius: 10px;
              box-shadow: 0 3px 14px rgba(0,0,0,.09); }}
 figure.gif img {{ border-color: #2b6cb0; box-shadow: 0 3px 16px rgba(43,108,176,.22); }}
 figcaption {{ font-size: 12.5px; color:#718096; margin-top: 6px; }}
 figure.gif figcaption {{ color:#2b6cb0; font-weight: 600; }}
 table {{ border-collapse: collapse; width: 100%; font-size: 13px; margin: 12px 0; }}
 th, td {{ border: 1px solid #cbd5e0; padding: 6px 9px; text-align: left; }}
 th {{ background: #edf2f7; }}
 code, kbd {{ background:#edf2f7; padding: 1px 6px; border-radius: 4px; font-size: 12.5px; }}
 kbd {{ border: 1px solid #cbd5e0; border-bottom-width: 2px; }}
 .tip {{ background:#f0fff4; border-left: 4px solid #2f855a; padding: 8px 14px; margin: 12px 0; }}
 .warn {{ background:#fffbeb; border-left: 4px solid #d69e2e; padding: 8px 14px; margin: 12px 0; }}
 .toc {{ background:#f7fafc; border:1px solid #e2e8f0; border-radius: 10px; padding: 14px 22px; }}
 .toc a {{ text-decoration: none; color:#2b6cb0; }}
 .miss {{ color:#c53030; font-size: 12px; }}
 .meta {{ color:#718096; font-size: 13px; }}
</style></head><body>

<h1>4WIS Simulator 使用说明书 <span style="font-size:15px;color:#718096">v{VER} · 图文版</span></h1>
<p class="meta">四轮独立转向（4WIS）仿真平台 —— 交互驾驶 · 批量实验 · 结果分析 · 负载选型 · 功能安全研究。
本说明书全部截图由 <code>python scripts/build_manual.py</code> 从当前版本实跑生成。</p>

<div class="toc"><b>目录</b><br>
<a href="#s1">1 安装启动</a> ·
<a href="#s2">2 界面总览：七段工作流</a> ·
<a href="#s3">3 运行页（驾驶工作台）</a> ·
<a href="#s4">4 手柄：六种映射模式与校准</a> ·
<a href="#s5">5 场景页</a> ·
<a href="#s6">6 车辆页</a> ·
<a href="#s7">7 试验页（批量实验）</a> ·
<a href="#s8">8 分析页（对比与回放）</a> ·
<a href="#s9">9 负载特性</a> ·
<a href="#s10">10 原理简介</a> ·
<a href="#s11">11 快捷键与命令面板</a> ·
<a href="#s12">12 工程工作流配方</a> ·
<a href="#s13">13 FAQ</a> ·
<a href="#s14">14 版本纪要 v0.9→v{VER}</a></div>

<h2 id="s1">1　安装启动</h2>
<p><b>便携版（推荐）</b>：解压对应平台的 zip → macOS 双击 <code>start.command</code>（首次被 Gatekeeper
拦截时右键→打开），Windows 双击 <code>start.bat</code> → 浏览器自动打开
<code>http://127.0.0.1:8010/</code>。无需安装 Python / Node。关掉终端窗口即停止。</p>
<p><b>开发态</b>：<code>backend</code> 下 <code>python -m uvicorn sim4wis.main:app --port 8010</code>，
<code>frontend</code> 下 <code>npm run dev</code>（Vite :5173 代理到 8010）。</p>
<p>首次进入有「快速开始」四步引导（右上 <kbd>?</kbd> 可随时唤回）：</p>
{img("00_quickstart", "首次进入：快速开始引导卡 + 运行页默认布局")}

<h2 id="s2">2　界面总览：七段工作流</h2>
<p>左侧竖排导航按 CarMaker 式工作流组织，覆盖从建模到分析的完整链路：</p>
<table>
<tr><th>段</th><th>做什么</th></tr>
<tr><td>🕹 <b>运行</b></td><td>交互驾驶工作台：键盘/手柄实时开、调策略调模型、录制曲线</td></tr>
<tr><td>🧪 <b>试验</b></td><td>定义可复现实验（机动分段×车速），一键批量跑「策略×车速」矩阵</td></tr>
<tr><td>📊 <b>分析</b></td><td>run 结果库：KPI 对比表、多 run 通道叠图、轨迹俯视、幽灵车回放</td></tr>
<tr><td>🚗 <b>车辆</b></td><td>车辆/悬架/转向几何参数编辑与项目（YAML）保存加载</td></tr>
<tr><td>🛣 <b>场景</b></td><td>场景路况（广场/小镇/赛道）、参考路径、扰动（冰面/对开/减速带/坡道）、故障注入</td></tr>
<tr><td>⚙ <b>负载</b></td><td>准静态转向负载扫描：齿条力/δ_eq/敏感度 → 电机与作动器选型</td></tr>
<tr><td>📖 <b>原理</b></td><td>模型数学原理讲解（坐标系→轮胎→主销力矩→bicycle 耦合），含交互演示</td></tr>
</table>
<p>顶栏：版本徽标、当前页名、车速/策略实时摘要、主题切换、连接状态。任意页面
<kbd>⌘K</kbd>/<kbd>Ctrl K</kbd> 唤出命令面板（见 §11）。</p>

<h2 id="s3">3　运行页（驾驶工作台）</h2>
<p>左侧 2D/3D 视图 + 右侧四个功能 tab（驾驶/设计/验证/数据）。2D 俯视图显示车身、
四轮实际转角、每轮转向中心线、整车瞬心与行驶轨迹；HUD 给出车速/横摆/位姿/四轮转角/μ/瞬心偏差。</p>
{img("01_run_overview", "键盘 W+A 驾驶后的运行页：轨迹、四轮转角、HUD 实时量")}
{gif("gif_drive", "键盘绕桩驾驶：正弦转向下四轮转角与轨迹的实时响应")}
<h3>3.1 视图</h3>
<p>右上 2D/3D 切换。3D 场景包含车身、可转向车轮、场景路面与轨迹带；相机可拖拽环绕。</p>
{img("02_view3d", "3D 视图")}
<h3>3.2 动力学模型（三档）</h3>
{img("03_model_panel", "模型选择面板")}
<table>
<tr><th>模型</th><th>适用</th></tr>
<tr><td><b>运动学</b></td><td>纯几何、无轮胎滑移——验证转角分配策略最干净</td></tr>
<tr><td><b>动力学</b>（默认主力）</td><td>3-DOF 车体+轮速伺服+线性/Pacejka 胎（摩擦圆、c_α(F_z) 载荷敏感、
阻力/升力/toe/camber）——工程曲线的默认口径</td></tr>
<tr><td><b>多体 14DOF</b></td><td>加悬架垂向/侧倾/俯仰 DOF——过减速带、载荷转移研究</td></tr>
</table>
<h3>3.3 控制策略（12 种）</h3>
{img("04_strategy_panel", "控制策略面板")}
<table>
<tr><th>策略</th><th>一句话</th></tr>
<tr><td>传统阿克曼</td><td>只前轴转向，后轮锁 0——基准对照</td></tr>
<tr><td>理想阿克曼</td><td>四轮共瞬心、零运动学侧偏——4WIS 招牌</td></tr>
<tr><td>后轮转向</td><td>五种子模式（定比/车速调度/横摆反馈/稳态+瞬态/模型跟踪）</td></tr>
<tr><td>蟹行</td><td>四轮同角平移（见下图演示）</td></tr>
<tr><td>零半径</td><td>绕几何中心原地自转</td></tr>
<tr><td>轨迹跟踪</td><td>纯追踪当前参考路径（场景页生成），带曲率限速</td></tr>
<tr><td>容错重构</td><td>单轮失效降级控制（镜像抵消/增益补偿+横摆 PI+限速，配合故障注入）</td></tr>
<tr><td>手柄直控 / 手柄全向</td><td>由手柄映射模式自动接管（§4）</td></tr>
<tr><td>Python / JS 策略</td><td>自定义控制律热加载（设计 tab）</td></tr>
</table>
{img("06_crab_drive", "蟹行演示：四轮同角、车身不旋转的斜向平移轨迹")}
{gif("gif_crab", "蟹行：四轮同角，整车不改航向地斜向平移")}
<h3>3.4 驾驶输入</h3>
{img("05_drive_panel", "驾驶输入面板：回读条、保持车速、定速巡航、回正速度")}
<ul>
<li><kbd>W</kbd>/<kbd>S</kbd> 油门（<b>目标车速</b>口径：throttle×v_max），<kbd>A</kbd>/<kbd>D</kbd> 转向，
<kbd>Space</kbd> 松油/急停，<kbd>R</kbd> 重置，<kbd>1–5</kbd> 快切策略。</li>
<li><b>保持车速</b>：W/S 变成增减持久目标，松手不减速——稳态工况必备。</li>
<li><b>定速巡航</b>：输入精确 km/h 数值锁定。</li>
<li><b>转向回正速度</b>：0=保持转角（稳态圆周），1.5×=快速回正。</li>
</ul>
<h3>3.5 设计 / 验证 / 数据 tab</h3>
<ul>
<li><b>设计</b>：策略设计器（k(vx) 调度曲线编辑）、Python/JS 自定义策略热加载。</li>
<li><b>验证</b>：开环激励（角阶跃/正弦/扫频/双移线）、策略评分（7 项 KPI 实时）、A/B 轨迹对比叠加。</li>
<li><b>数据</b>：两点测距、通道录制与 CSV 导出、动作脚本、实时曲线（转角/主销力矩/齿条力/侧偏角…）。</li>
</ul>

<h2 id="s4">4　手柄：六种映射模式与校准</h2>
<p>「驾驶」tab 底部的 <b>🎮 手柄映射与校准</b> 面板。接入任意标准手柄/USB 方向盘即用；
核心是一套「<b>轴 → 车轮分组</b>」绑定机制——分组方式本身就是模式：</p>
{img("07_gp_panel", "手柄映射与校准面板：模式预设、通道绑定、死区/expo/反向、实时监视")}
<table>
<tr><th>模式</th><th>左摇杆</th><th>右摇杆</th><th>接管策略</th><th>玩法</th></tr>
<tr><td><b>经典</b>（默认）</td><td>X=转向</td><td>—</td><td>跟随当前策略</td><td>扳机油门，与键盘叠加</td></tr>
<tr><td><b>前后轴独立</b></td><td>X=前轴角</td><td>X=后轴角</td><td>manual_wheel</td><td>反相小半径 / 同相蟹行全靠手</td></tr>
<tr><td><b>左右侧独立</b></td><td>X=左侧两轮</td><td>X=右侧两轮</td><td>manual_wheel</td><td>左右差动转向实验</td></tr>
<tr><td><b>逐轮直控</b></td><td>X=FL Y=RL</td><td>X=FR Y=RR</td><td>manual_wheel</td><td>四轮完全独立，纯手感研究</td></tr>
<tr><td><b>蟹行</b></td><td>X=四轮同角</td><td>—</td><td>manual_wheel</td><td>纯横移</td></tr>
<tr><td><b>全向车身</b></td><td>平移(前+横)</td><td>X=自转</td><td>manual_body</td><td>斜开+自转同时做</td></tr>
</table>
<p><b>效果演示</b>（下列截图为各模式注入摇杆输出后的真实车轮响应）：</p>
{img("08_gp_frontrear", "前后轴独立：前轴 +17.5°、后轴 −17.5°（反相 → 最小转弯半径姿态）")}
{gif("gif_frontrear", "前后轴独立：左右摇杆分别把前轴、后轴反相扫角，两轴完全解耦")}
{img("09_gp_perwheel", "逐轮直控：四个车轮四个不同转角，完全独立")}
{gif("gif_perwheel", "逐轮直控：四轮各自不同相位摆动，互不影响")}
{img("10_gp_holonomic", "全向车身：斜向平移的同时自转——普通车做不到的完整平面自由度")}
{gif("gif_holonomic", "全向车身：车身一边斜向平移、一边持续自转")}
<h3>4.1 校准与绑定（换任何设备都能用）</h3>
<ul>
<li><b>绑定</b>：点某通道的「绑定」按钮 → 拨动你想用的摇杆/踏板 → 自动识别轴号。方向盘、
HOTAS 等非标准轴序设备靠这个即插即用。</li>
<li><b>每轴</b>：反向勾选、expo 灵敏度曲线（中心更细腻）。<b>全局</b>：死区、转向灵敏度。</li>
<li><b>油门来源</b>：扳机 RT/LT（默认）或任意摇杆轴（如方向盘踏板）。</li>
<li><b>实时监视</b>：面板底部显示所有轴/键的实时数值条，绑定是否正确一眼可见。</li>
<li>配置自动存本机（localStorage）。选直控/全向预设会<b>自动切换后端策略</b>，切回「经典」自动恢复理想阿克曼。</li>
</ul>
<div class="tip">推荐上手顺序：经典模式熟悉车 → 前后轴独立体会 4WIS 的前后解耦 → 全向车身感受完整平面自由度。</div>

<h2 id="s5">5　场景页</h2>
<p>视图 + 四个编辑面板：<b>场景路况</b>（广场/小镇/小赛道/上赛近似，加载后车辆移到起点，2D/3D 同步渲染）、
<b>轨迹/路径</b>（模板：直线/圆弧/绕桩/双移线/八字/侧方位，或点击画布手动绘制；「跟踪此路径」一键交给
轨迹跟踪策略）、<b>路面扰动</b>（冰面/对开路面/减速带/坡道，选类型后点画布放置，参数可改可删）、
<b>故障注入</b>（执行器卡死/受限、传感器偏置/噪声/丢失——研究单轮失效时配合「容错重构」策略）。</p>
{img("11_scene_page", "场景页：小镇场景 + 双移线参考路径（含锥桶）")}
{img("12_disturbance", "扰动示例：减速带（横条）与低附着冰面（色块），车轮驶过时逐轮生效")}
{img("13_fault_panel", "故障注入面板")}

<h2 id="s6">6　车辆页</h2>
<p>左列参数组（车身/悬架主销/轮胎/转向传动/气动……全部 SI 单位，改动即时生效并重置模型），
右列项目管理（当前 车辆+策略+场景 快照存为 YAML，可加载/分发；内置 LS9 等预置）。</p>
{img("14_vehicle_page", "车辆页：参数编辑 + 项目保存/加载")}

<h2 id="s7">7　试验页（批量实验）</h2>
<p>把「机动 × 车速 × 策略」定义成<b>可复现实验</b>并批量运行——离线全速（约 25× 实时），
每个 run 落盘可回放：</p>
<ol>
<li><b>实验库</b>（左）：内置 ISO 双移线、角阶跃示例；点击载入，可另存。</li>
<li><b>实验定义</b>（中）：机动分段编辑——每段有时长、目标车速+斜坡、转向剖面
（恒值/角阶跃/斜坡/单频正弦/扫频/双移线）；可选参考路径模板与记录频率。</li>
<li><b>运行矩阵</b>（右）：勾选多个策略 × 填多个车速 → 自动展开变体；点运行看进度条，
完成即出 KPI 小表，一键「去分析页对比」。</li>
</ol>
{img("15_experiment_page", "试验页：载入 ISO 双移线示例，2 策略矩阵跑完直接出 KPI")}
<div class="tip">转向幅值是归一化输入（理想阿克曼下≈曲率分数）：60 km/h 时 0.05 ≈ 4.6 m/s² 侧向加速度，
&gt;0.06 会超附着极限变甩尾工况；目标车速务必配 2–4 s 斜坡。</div>

<h2 id="s8">8　分析页（对比与回放）</h2>
<p>run 结果库（左，调色板多选 ≤6）→ <b>KPI 对比表</b>（横摆峰值/齿条力/瞬心偏差/能耗/阶跃响应组…）
→ <b>通道叠图</b>（预设 chips 或下拉加图，可导 PNG）→ <b>轨迹俯视</b>（等比例）→ <b>回放</b>。</p>
{img("16_analysis_kpi", "KPI 对比表：4 个 run 并排，色点对应左侧选择")}
{img("17_analysis_charts", "通道叠图与轨迹俯视：多 run 同图对比")}
<p><b>回放</b>：所选 runs 以幽灵车叠放重演（车轮显示记录的实际转角），播放/暂停/0.5–4× 倍速/时间轴拖动；
回放时间同步在所有曲线上画黄色游标——「动画与曲线共游标」。</p>
{img("18_replay", "幽灵车回放 + 曲线黄游标联动")}

<h2 id="s9">9　负载特性</h2>
<p>准静态扫描（车速 × 目标转角）算每轮转向阻力矩 → 齿条力 → 电机力矩：选电机/作动器、
看零输出自然转角 δ_eq 随速漂移、做参数敏感度。顶栏「受力口径」切换
<b>整车装载</b>（bicycle 耦合，看手感/δ_eq）与<b>单轮台架</b>（α=−δ，选型最差工况）。</p>
{img("19_load_page", "负载特性页：τ-δ / 齿条力曲线族与 KPI 卡片")}

<h2 id="s10">10　原理简介</h2>
<p>从坐标系与轮胎模型讲到主销力矩与 bicycle 耦合的完整推导（含交互演示），PM 能看懂、
工程师可引用；与代码同源同口径。</p>
{img("20_theory_page", "原理简介页")}

<h2 id="s11">11　快捷键与命令面板</h2>
<table>
<tr><th>键</th><th>作用</th></tr>
<tr><td><kbd>W A S D</kbd> / <kbd>Space</kbd></td><td>驾驶 / 松油急停</td></tr>
<tr><td><kbd>R</kbd></td><td>重置位姿与轨迹</td></tr>
<tr><td><kbd>1–5</kbd></td><td>快切策略</td></tr>
<tr><td><kbd>⌘K</kbd> / <kbd>Ctrl K</kbd></td><td>命令面板：页面导航/策略/模型/主题/重置，输入过滤 + ↑↓ + 回车</td></tr>
</table>
{img("21_cmdk", "命令面板：输入「试验」直接跳页")}

<h2 id="s12">12　工程工作流配方</h2>
<h3>① 转向电机/作动器选型</h3>
<p>负载页选「单轮台架」口径 → 扫车速×转角 → 峰值齿条力 ×（r_p/i）换算电机力矩 →
敏感度面板看主销几何的影响；持续工作点用安全研究的镜像抵消工况校核（RL/RR 对抗侧偏角驻留）。</p>
<h3>② 控制策略对比</h3>
<p>试验页：同一机动 × 多策略矩阵一键跑 → 分析页 KPI 表排序 + 叠图看瞬态差异 + 回放看轨迹差异。
比旧的「工作台手动 A/B」快一个量级。</p>
<h3>③ 单轮失效功能安全研究（可完整复现）</h3>
<p><code>python scripts/study_single_wheel_failure.py</code> 一键复现 ISO 26262 可控性研究
（~170 runs：前轮自由脚轮/后轮自锁锁死 × 三工况 × 缓解 × 9 参数敏感性），
报告在 <code>docs/reports/single_wheel_failure_safety_analysis.html</code>。
交互复现：场景页注入故障 + 策略选「容错重构」。</p>
<h3>④ 手柄人因/手感实验</h3>
<p>§4 的直控模式 + 数据 tab 录制 → 分析页回放，可做「人开 vs 算法」的同屏对比。</p>

<h2 id="s13">13　FAQ</h2>
<table>
<tr><th>问题</th><th>答案</th></tr>
<tr><td>油门像"定速"不像踏板？</td><td>设计如此：throttle×v_max=目标车速（轮速伺服闭环），扫工况更稳。</td></tr>
<tr><td>手柄没反应？</td><td>先按任意键/扳机激活设备（浏览器安全策略）；面板底部监视条应有跳动；
不对就逐通道重新「绑定」。</td></tr>
<tr><td>直行时轮子有 ±0.1° 转角、主销力矩非零？</td><td>静态 toe 与 camber 推力的真实体现（v0.9 起时域与负载页口径统一）。</td></tr>
<tr><td>8010 端口被占？</td><td>本机若跑其它服务（如本地 LLM 常占 8000）不冲突；8010 被占时改
<code>start</code> 脚本里的端口。</td></tr>
<tr><td>分析页 run 从哪来？</td><td>试验页跑的批量实验（含安全研究脚本产生的）都会落盘到 runs/。</td></tr>
</table>

<h2 id="s14">14　版本纪要 v0.9 → v{VER}</h2>
<table>
<tr><th>版本</th><th>要点</th></tr>
<tr><td>v0.9.0</td><td>时域/负载页物理口径统一（阻力/升力/toe/camber）；轮速积分半隐式化修复中低速潜伏失稳；手柄初版</td></tr>
<tr><td>v0.10.0</td><td>实验底座：无头会话（25× 实时）、run 落盘、KPI 后端化、变体矩阵；轮速伺服加前馈修斜坡超调</td></tr>
<tr><td>v0.11.x</td><td>七段工作流导航、试验页、分析页、幽灵车回放、⌘K 命令面板</td></tr>
<tr><td>v0.12.0</td><td>时域 c_α(F_z) 载荷敏感度</td></tr>
<tr><td>v0.13–0.14</td><td>单轮失效 ISO 26262 研究：故障注入（含自由脚轮机构 ODE）、容错重构策略、参数敏感性流水线、论文级报告</td></tr>
<tr><td>v{VER}</td><td>手柄映射机制：六模式预设（前后轴/左右侧/逐轮/蟹行/全向）+ 点击绑定校准 + 死区/expo/反向</td></tr>
</table>
<p class="meta">完整变更见仓库 CHANGELOG.md · 本说明书由 scripts/build_manual.py 自动生成于 v{VER}</p>
</body></html>"""
    OUT_HTML.write_text(html, encoding="utf-8")
    size_mb = OUT_HTML.stat().st_size / 1e6
    print(f"✓ 说明书：{OUT_HTML}（{size_mb:.1f} MB）")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-capture", action="store_true")
    args = ap.parse_args()
    if not args.skip_capture:
        print("§1 截图巡游 …")
        ok = capture()
        n = sum(ok.values())
        print(f"  截图 {n}/{len(ok)} 成功")
    print("§2 组装 HTML …")
    build_html()


if __name__ == "__main__":
    main()
