/**
 * CommandPalette — ⌘K / Ctrl+K global command launcher (Phase B-2).
 *
 * One keystroke to: jump between workflow pages, switch strategy / dynamics
 * model, toggle theme, reset the sim. Substring filter over label + latin
 * keywords, ↑/↓ + Enter keyboard driving, Esc closes.
 */

import { useEffect, useMemo, useRef, useState } from "react";

import { resetSim, setModel, setStrategy } from "@/api/ws";
import { AppPage, useSimStore } from "@/store/sim";

interface Command {
  id: string;
  group: string;
  label: string;
  keywords: string;      // latin search terms (page ids, strategy names…)
  run: () => void;
}

const PAGES: { id: AppPage; label: string }[] = [
  { id: "run", label: "运行（驾驶工作台）" },
  { id: "experiment", label: "试验（实验/批量矩阵）" },
  { id: "analysis", label: "分析（run 对比/回放）" },
  { id: "vehicle", label: "车辆（参数/项目）" },
  { id: "scene", label: "场景（路径/扰动/故障）" },
  { id: "load", label: "负载特性" },
  { id: "model", label: "原理简介" },
];

const MODELS = [
  { id: "kinematic", label: "运动学" },
  { id: "simplified_dynamic", label: "动力学" },
  { id: "multibody", label: "多体(14DOF)" },
];

export default function CommandPalette() {
  const [open, setOpen] = useState(false);
  const [query, setQuery] = useState("");
  const [idx, setIdx] = useState(0);
  const inputRef = useRef<HTMLInputElement>(null);
  const strategies = useSimStore((s) => s.strategies);
  const setPage = useSimStore((s) => s.setPage);
  const theme = useSimStore((s) => s.theme);
  const setTheme = useSimStore((s) => s.setTheme);
  const pushToast = useSimStore((s) => s.pushToast);

  // Global hotkey.
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === "k") {
        e.preventDefault();
        setOpen((v) => !v);
        setQuery("");
        setIdx(0);
      } else if (e.key === "Escape") {
        setOpen(false);
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  useEffect(() => {
    if (open) window.setTimeout(() => inputRef.current?.focus(), 30);
  }, [open]);

  const commands = useMemo<Command[]>(() => {
    const close = () => setOpen(false);
    const cmds: Command[] = [];
    for (const p of PAGES) {
      cmds.push({
        id: `page:${p.id}`, group: "导航", label: `去 ${p.label}`,
        keywords: `go page ${p.id}`,
        run: () => { setPage(p.id); close(); },
      });
    }
    for (const s of strategies) {
      cmds.push({
        id: `strategy:${s}`, group: "策略", label: `切换策略：${s}`,
        keywords: `strategy ${s}`,
        run: () => { setStrategy(s); pushToast("info", `策略 → ${s}`); close(); },
      });
    }
    for (const m of MODELS) {
      cmds.push({
        id: `model:${m.id}`, group: "模型", label: `切换模型：${m.label}`,
        keywords: `model ${m.id}`,
        run: () => {
          setModel(m.id).then(() => pushToast("info", `模型 → ${m.label}`))
            .catch((e) => pushToast("error", `切换失败：${e.message}`));
          close();
        },
      });
    }
    cmds.push({
      id: "sim:reset", group: "仿真", label: "重置位姿与轨迹（R）",
      keywords: "reset pose",
      run: () => { resetSim(); pushToast("info", "已重置"); close(); },
    });
    cmds.push({
      id: "ui:theme", group: "界面", label: `切换为${theme === "dark" ? "浅色" : "深色"}主题`,
      keywords: "theme dark light",
      run: () => { setTheme(theme === "dark" ? "light" : "dark"); close(); },
    });
    return cmds;
  }, [strategies, theme, setPage, setTheme, pushToast]);

  const q = query.trim().toLowerCase();
  const filtered = q
    ? commands.filter((c) => (c.label + " " + c.keywords).toLowerCase().includes(q))
    : commands;
  const sel = Math.min(idx, Math.max(0, filtered.length - 1));

  const onInputKey = (e: React.KeyboardEvent) => {
    if (e.key === "ArrowDown") { e.preventDefault(); setIdx((i) => Math.min(i + 1, filtered.length - 1)); }
    else if (e.key === "ArrowUp") { e.preventDefault(); setIdx((i) => Math.max(i - 1, 0)); }
    else if (e.key === "Enter") { e.preventDefault(); filtered[sel]?.run(); }
  };

  if (!open) return null;

  let lastGroup = "";
  return (
    <div className="cmdk-overlay" onMouseDown={() => setOpen(false)}>
      <div className="cmdk-panel" onMouseDown={(e) => e.stopPropagation()}>
        <input
          ref={inputRef}
          className="cmdk-input"
          placeholder="输入命令…（页面 / 策略 / 模型 / 主题）"
          value={query}
          onChange={(e) => { setQuery(e.target.value); setIdx(0); }}
          onKeyDown={onInputKey}
        />
        <div className="cmdk-list">
          {filtered.map((c, i) => {
            const showGroup = c.group !== lastGroup;
            lastGroup = c.group;
            return (
              <div key={c.id}>
                {showGroup && <div className="cmdk-group">{c.group}</div>}
                <button
                  className={`cmdk-item ${i === sel ? "active" : ""}`}
                  onMouseEnter={() => setIdx(i)}
                  onClick={() => c.run()}
                >
                  {c.label}
                </button>
              </div>
            );
          })}
          {filtered.length === 0 && <div className="wf-empty">无匹配命令</div>}
        </div>
        <div className="cmdk-hint">↑↓ 选择 · Enter 执行 · Esc 关闭 · ⌘K 唤起</div>
      </div>
    </div>
  );
}
