import { lazy, Suspense, useEffect, useState } from "react";
import type { ReactNode } from "react";

import Viewport from "@/components/Viewport";
import ControlPanel from "@/components/ControlPanel";
import ChartPanel from "@/components/ChartPanel";
import KeyboardInput from "@/components/KeyboardInput";
import RecordingPanel from "@/components/RecordingPanel";
import ScriptPanel from "@/components/ScriptPanel";
import TrajectoryPanel from "@/components/TrajectoryPanel";
import MeasurePanel from "@/components/MeasurePanel";
import DisturbancePanel from "@/components/DisturbancePanel";
import ComparePanel from "@/components/ComparePanel";
import FaultPanel from "@/components/FaultPanel";
import UserPythonPanel from "@/components/UserPythonPanel";
import JsStrategyPanel from "@/components/JsStrategyPanel";
import StrategyDesignerPanel from "@/components/StrategyDesignerPanel";
import ScenarioPanel from "@/components/ScenarioPanel";
import ExcitationPanel from "@/components/ExcitationPanel";
import ScorePanel from "@/components/ScorePanel";
import CommandPalette from "@/components/CommandPalette";
import QuickStartCard from "@/components/QuickStartCard";
import ErrorBoundary from "@/components/ErrorBoundary";
import Toasts from "@/components/Toasts";
import { connectSimSocket, fetchPath, fetchScenario } from "@/api/ws";
import { fetchJSON } from "@/api/http";
import { AppPage, useSimStore } from "@/store/sim";

const LoadAnalysisPage = lazy(() => import("@/components/LoadAnalysisPage"));
const ModelTheoryPage = lazy(() => import("@/components/ModelTheoryPage"));
const ExperimentPage = lazy(() => import("@/components/ExperimentPage"));
const AnalysisPage = lazy(() => import("@/components/AnalysisPage"));
const VehicleGeometryStudio = lazy(() => import("@/components/vehicle/VehicleGeometryStudio"));

// Workbench sidebar tab groups (scene editing moved to the 场景 page).
// All panels stay mounted (timers / WS subscriptions keep running across tab
// switches); inactive groups are hidden via CSS, not unmounted.
const TABS = [
  { id: "drive", label: "驾驶", hint: "策略 / 模型 / 油门 / 路面 / 仿真控制" },
  { id: "design", label: "设计", hint: "策略设计器 / Python·JS 策略" },
  { id: "validate", label: "验证", hint: "开环激励 / 评分 / A/B 对比" },
  { id: "data", label: "数据", hint: "测量 / 录制 / 脚本 / 实时曲线" },
] as const;
type TabId = (typeof TABS)[number]["id"];

// CarMaker-style workflow rail: 建模 → 场景 → 试验 → 运行 → 分析 → 知识.
const RAIL: { id: AppPage; icon: string; label: string; hint: string }[] = [
  { id: "run", icon: "🕹", label: "运行", hint: "交互驾驶工作台（键盘/手柄实时仿真）" },
  { id: "experiment", icon: "🧪", label: "试验", hint: "定义可复现实验，批量运行策略×车速矩阵" },
  { id: "analysis", icon: "📊", label: "分析", hint: "run 结果库：KPI 对比 / 多 run 叠图 / 轨迹" },
  { id: "vehicle", icon: "🚗", label: "车辆", hint: "车辆/悬架/转向几何参数与项目管理" },
  { id: "scene", icon: "🛣", label: "场景", hint: "参考路径 / 扰动（冰面·减速带·坡道）/ 故障注入" },
  { id: "load", icon: "⚙", label: "负载", hint: "准静态负载特性（齿条力/δ_eq/敏感度）" },
  { id: "model", icon: "📖", label: "原理", hint: "数学模型与理论简介" },
];

function TabGroup({ id, tab, children }: { id: TabId; tab: TabId; children: ReactNode }) {
  return <div style={{ display: tab === id ? "contents" : "none" }}>{children}</div>;
}

const QUICKSTART_KEY = "4wis_quickstart_dismissed";

function PageLoader() {
  return (
    <main className="page-loader" aria-live="polite">
      <span className="mono">Loading</span>
    </main>
  );
}

export default function App() {
  const online = useSimStore((s) => s.online);
  const strategy = useSimStore((s) => s.state?.strategy ?? "—");
  const vx = useSimStore((s) => s.state?.velocity?.vx ?? 0);
  const vy = useSimStore((s) => s.state?.velocity?.vy ?? 0);
  const setStrategies = useSimStore((s) => s.setStrategies);
  const pathVersion = useSimStore((s) => s.pathVersion);
  const scenarioVersion = useSimStore((s) => s.scenarioVersion);
  const theme = useSimStore((s) => s.theme);
  const setTheme = useSimStore((s) => s.setTheme);
  const page = useSimStore((s) => s.page);
  const setPage = useSimStore((s) => s.setPage);
  const [tab, setTab] = useState<TabId>("drive");
  const [version, setVersion] = useState<string | null>(null);
  const [quickStart, setQuickStart] = useState(
    () => localStorage.getItem(QUICKSTART_KEY) !== "1",
  );

  const speedKmh = Math.hypot(vx, vy) * 3.6;

  const dismissQuickStart = () => {
    setQuickStart(false);
    try {
      localStorage.setItem(QUICKSTART_KEY, "1");
    } catch {
      /* ignore storage failures (private mode) */
    }
  };

  // Apply theme to the document root (drives the CSS variables).
  useEffect(() => {
    document.documentElement.dataset.theme = theme;
  }, [theme]);

  // Connect WebSocket on mount.
  useEffect(() => {
    connectSimSocket();
  }, []);

  // Pull strategy list once from REST.
  useEffect(() => {
    fetchJSON<{ strategies: string[] }>("/api/strategies")
      .then((d) => setStrategies(d.strategies ?? []))
      .catch(() => setStrategies([]));
  }, [setStrategies]);

  // Backend version for the header badge.
  useEffect(() => {
    fetchJSON<{ version: string }>("/api/version")
      .then((d) => setVersion(d.version ?? null))
      .catch(() => setVersion(null));
  }, []);

  // Re-fetch the reference path whenever the backend bumps path_version.
  useEffect(() => {
    if (pathVersion >= 0) fetchPath().catch(() => undefined);
  }, [pathVersion]);

  // Re-fetch the scenario geometry whenever the backend bumps scenario_version.
  useEffect(() => {
    if (scenarioVersion >= 0) fetchScenario().catch(() => undefined);
  }, [scenarioVersion]);

  return (
    <div className="app-shell">
      <header className="app-header">
        <span className="brand">4WIS</span>
        <span className="title">Simulator</span>
        {version && (
          <span
            className="small mono"
            style={{ color: "var(--muted)", fontSize: 10, alignSelf: "flex-end", paddingBottom: 2 }}
            title="后端版本（/api/version）"
          >
            v{version}
          </span>
        )}
        <span className="page-title">{RAIL.find((r) => r.id === page)?.label ?? ""}</span>
        <div className="header-summary" aria-label="当前状态摘要">
          <span className="hs-item">
            <span className="hs-k">车速</span>
            <span className="hs-v">{speedKmh.toFixed(1)}<i>km/h</i></span>
          </span>
          <span className="hs-item">
            <span className="hs-k">策略</span>
            <span className="hs-v" title={strategy}>{strategy}</span>
          </span>
        </div>
        {page === "run" && (
          <button
            className="quickstart-reopen"
            onClick={() => setQuickStart(true)}
            title="重新打开快速开始"
            aria-label="快速开始"
          >
            ?
          </button>
        )}
        <button
          className="theme-toggle"
          onClick={() => setTheme(theme === "dark" ? "light" : "dark")}
          title="切换深色/浅色主题"
        >
          {theme === "dark" ? "☀ 浅色" : "🌙 深色"}
        </button>
        <span className={`status ${online ? "online" : "offline"}`}>
          {online ? "● 已连接" : "○ 未连接"}
        </span>
      </header>

      <div className="app-body">
        {/* ── workflow rail ── */}
        <nav className="nav-rail" aria-label="工作流">
          {RAIL.map((r) => (
            <button
              key={r.id}
              className={`rail-item ${page === r.id ? "active" : ""}`}
              title={r.hint}
              onClick={() => setPage(r.id)}
            >
              <span className="rail-icon">{r.icon}</span>
              <span className="rail-label">{r.label}</span>
            </button>
          ))}
        </nav>

        {/* ── pages ── */}
        {page === "run" && (
          <main className="app-main">
            <section className="viewport-pane">
              <ErrorBoundary label="视图">
                <Viewport />
              </ErrorBoundary>
              {quickStart && (
                <QuickStartCard
                  onClose={dismissQuickStart}
                  onGoTab={(t) => setTab(t as TabId)}
                  onGoPage={(p) => setPage(p === "sim" ? "run" : (p as AppPage))}
                />
              )}
            </section>

            <aside className="side-pane">
              <nav className="side-tabs" role="tablist" aria-label="功能分组">
                {TABS.map((t) => (
                  <button
                    key={t.id}
                    role="tab"
                    aria-selected={tab === t.id}
                    className={`side-tab ${tab === t.id ? "active" : ""}`}
                    title={t.hint}
                    onClick={() => setTab(t.id)}
                  >
                    {t.label}
                  </button>
                ))}
              </nav>
              <div className="side-scroll">
                <ErrorBoundary label="侧栏">
                  <TabGroup id="drive" tab={tab}>
                    <ControlPanel />
                  </TabGroup>
                  <TabGroup id="design" tab={tab}>
                    <StrategyDesignerPanel />
                    <UserPythonPanel />
                    <JsStrategyPanel />
                  </TabGroup>
                  <TabGroup id="validate" tab={tab}>
                    <ExcitationPanel />
                    <ScorePanel />
                    <ComparePanel />
                  </TabGroup>
                  <TabGroup id="data" tab={tab}>
                    <MeasurePanel />
                    <RecordingPanel />
                    <ScriptPanel />
                    <ChartPanel />
                  </TabGroup>
                </ErrorBoundary>
              </div>
            </aside>
          </main>
        )}

        <Suspense fallback={<PageLoader />}>
          {page === "experiment" && (
            <ErrorBoundary label="试验">
              <ExperimentPage />
            </ErrorBoundary>
          )}

          {page === "analysis" && (
            <ErrorBoundary label="分析">
              <AnalysisPage />
            </ErrorBoundary>
          )}

          {page === "vehicle" && (
            <ErrorBoundary label="车辆">
              <VehicleGeometryStudio />
            </ErrorBoundary>
          )}
        </Suspense>

        {page === "scene" && (
          <main className="app-main">
            <section className="viewport-pane">
              <ErrorBoundary label="场景视图">
                <Viewport />
              </ErrorBoundary>
            </section>
            <aside className="side-pane">
              <div className="side-scroll" style={{ paddingTop: 8 }}>
                <ErrorBoundary label="场景编辑">
                  <ScenarioPanel />
                  <TrajectoryPanel />
                  <DisturbancePanel />
                  <FaultPanel />
                </ErrorBoundary>
              </div>
            </aside>
          </main>
        )}

        <Suspense fallback={<PageLoader />}>
          {page === "load" && (
            <ErrorBoundary label="负载特性">
              <LoadAnalysisPage />
            </ErrorBoundary>
          )}

          {page === "model" && (
            <ErrorBoundary label="原理简介">
              <ModelTheoryPage />
            </ErrorBoundary>
          )}
        </Suspense>
      </div>

      {/* Listens to window-level keyboard events and pushes driver input */}
      <KeyboardInput />
      <CommandPalette />
      <Toasts />
    </div>
  );
}
