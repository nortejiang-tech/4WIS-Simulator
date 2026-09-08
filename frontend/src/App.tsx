import { lazy, Suspense, useEffect, useState } from "react";
import type { ReactNode } from "react";

import Viewport from "@/components/Viewport";
import ControlPanel from "@/components/ControlPanel";
import KeyboardInput from "@/components/KeyboardInput";
import CommandPalette from "@/components/CommandPalette";
import QuickStartCard from "@/components/QuickStartCard";
import ErrorBoundary from "@/components/ErrorBoundary";
import Toasts from "@/components/Toasts";
import { connectSimSocket, fetchPath, fetchScenario } from "@/api/ws";
import { fetchJSON } from "@/api/http";
import { AppPage, useSimStore } from "@/store/sim";
import "./App.css";
import InteractionModeBar from "@/components/InteractionModeBar";
import ManualLifecycle from "@/components/ManualLifecycle";
import { postJSON } from "@/api/http";

const AgentWorkspace = lazy(() => import("@/components/AgentWorkspace"));
const ScriptWorkspace = lazy(() => import("@/components/ScriptWorkspace"));

const LoadAnalysisPage = lazy(() => import("@/components/LoadAnalysisPage"));
const ModelTheoryPage = lazy(() => import("@/components/ModelTheoryPage"));
const ExperimentPage = lazy(() => import("@/components/ExperimentPage"));
const AnalysisPage = lazy(() => import("@/components/AnalysisPage"));
const VehicleGeometryStudio = lazy(() => import("@/components/vehicle/VehicleGeometryStudio"));
const ChartPanel = lazy(() => import("@/components/ChartPanel"));
const RecordingPanel = lazy(() => import("@/components/RecordingPanel"));
const ScriptPanel = lazy(() => import("@/components/ScriptPanel"));
const TrajectoryPanel = lazy(() => import("@/components/TrajectoryPanel"));
const MeasurePanel = lazy(() => import("@/components/MeasurePanel"));
const DisturbancePanel = lazy(() => import("@/components/DisturbancePanel"));
const ComparePanel = lazy(() => import("@/components/ComparePanel"));
const FaultPanel = lazy(() => import("@/components/FaultPanel"));
const UserPythonPanel = lazy(() => import("@/components/UserPythonPanel"));
const JsStrategyPanel = lazy(() => import("@/components/JsStrategyPanel"));
const StrategyDesignerPanel = lazy(() => import("@/components/StrategyDesignerPanel"));
const ScenarioPanel = lazy(() => import("@/components/ScenarioPanel"));
const ExcitationPanel = lazy(() => import("@/components/ExcitationPanel"));
const ScorePanel = lazy(() => import("@/components/ScorePanel"));
const GripPanel = lazy(() => import("@/components/GripPanel"));

// Workbench sidebar tab groups (scene editing moved to the 场景 page).
// Non-default groups load the first time they are opened, then stay mounted
// (timers / WS subscriptions keep running across later tab switches).
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
  { id: "agent", icon: "⌘", label: "Agent", hint: "独立仿真会话 / MCP 与 OpenAPI / 完整结果" },
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

function formatError(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

function PageLoader() {
  return (
    <main className="page-loader" aria-live="polite">
      <span className="app-mono">Loading</span>
    </main>
  );
}

function PanelLoader({ label }: { label: string }) {
  return (
    <div className="panel" aria-live="polite">
      <div className="panel-head">
        <span className="panel-name">{label}</span>
        <span className="app-xs app-mono" style={{ color: "var(--muted)", marginLeft: "auto" }}>
          Loading
        </span>
      </div>
    </div>
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
  const pushToast = useSimStore((s) => s.pushToast);
  const source = useSimStore(s => s.state?.interaction?.source ?? "idle");
  const gamepadId = useSimStore(s => s.gamepadId);
  const [tab, setTab] = useState<TabId>("drive");
  const [visitedTabs, setVisitedTabs] = useState<TabId[]>(["drive"]);
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

  const activateTab = (next: TabId) => {
    setVisitedTabs((cur) => (cur.includes(next) ? cur : [...cur, next]));
    setTab(next);
  };

  const tabVisited = (id: TabId) => visitedTabs.includes(id);

  // Apply theme to the document root (drives the CSS variables).
  useEffect(() => {
    document.documentElement.dataset.theme = theme;
  }, [theme]);

  // Connect WebSocket on mount.
  useEffect(() => {
    connectSimSocket();
    const runId = new URLSearchParams(window.location.search).get("run");
    if (runId) {
      useSimStore.getState().setAnalysisPreselect([runId]);
      useSimStore.getState().setPage("analysis");
    }
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
    if (pathVersion >= 0) {
      fetchPath().catch((e) => pushToast("error", `刷新参考路径失败：${formatError(e)}`));
    }
  }, [pathVersion, pushToast]);

  // Re-fetch the scenario geometry whenever the backend bumps scenario_version.
  useEffect(() => {
    if (scenarioVersion >= 0) {
      fetchScenario().catch((e) => pushToast("error", `刷新场景几何失败：${formatError(e)}`));
    }
  }, [scenarioVersion, pushToast]);

  return (
    <div className="app-shell">
      <header className="app-header">
        <span className="brand">4WIS</span>
        <span className="title">Simulator</span>
        {version && (
          <span
            className="app-xs app-mono app-version"
            style={{ color: "var(--muted)", fontSize: 10, alignSelf: "flex-end", paddingBottom: 2 }}
            title="后端版本（/api/version）"
          >
            v{version}
          </span>
        )}
        {/* Validity boundary, stated in the product rather than only in a
            document. Until a bench or vehicle dataset lands, a colleague
            handed a tool this complete will otherwise assume it has been
            correlated against a real car. Non-dismissible on purpose. */}
        <span
          className="validity-badge"
          title="本工具经解析闭式解与内部一致性验证（golden 基线逐位可复现），
但尚未与实车或台架数据做相关性验证。转向系统参数为工程估计值。
详见 docs/v2_steering_platform_plan.md §6。"
        >
          未经实车验证
        </span>
        <span className="page-title">{page === "script" ? "脚本工况" : RAIL.find((r) => r.id === page)?.label ?? ""}</span>
        <div className="header-summary" aria-label="当前状态摘要">
          {page === "agent" ? <span className="app-xs" style={{ color: "var(--muted)" }}>状态以所选 Agent 会话为准</span> : <>
          <span className="hs-item">
            <span className="hs-k">车速</span>
            <span className="hs-v">{speedKmh.toFixed(1)}<i>km/h</i></span>
          </span>
          <span className="hs-item">
            <span className="hs-k">策略</span>
            <span className="hs-v" title={strategy}>{strategy}</span>
          </span>
          </>}
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

      <InteractionModeBar
        mode={page === "run" ? "manual" : page === "script" || page === "experiment" ? "script" : page === "agent" ? "agent" : null}
        onChange={mode => { setPage(mode === "manual" ? "run" : mode === "script" ? "script" : "agent"); }}
        sourceLabel={page === "agent" ? "独立 Agent 会话" : source === "script" ? "实时脚本" : source === "manual" ? (gamepadId ? "键盘 / 手柄·方向盘" : "键盘") : "输入已释放"}
        connected={online}
        onStop={page === "run" || page === "script" ? () => {
          useSimStore.getState().requestZero(); useSimStore.getState().setManualArmed(false);
          postJSON("/api/interaction/control", { action: "stop" })
            .catch(err => pushToast("error", `停止输入失败：${String(err)}`));
        } : undefined}
      />

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
                  onGoTab={(t) => activateTab(t as TabId)}
                  onGoPage={(p) => setPage(p === "sim" ? "run" : (p as AppPage))}
                />
              )}
            </section>

            <aside className="side-pane">
              <ManualLifecycle />
              <nav className="side-tabs" role="tablist" aria-label="功能分组">
                {TABS.map((t) => (
                  <button
                    key={t.id}
                    role="tab"
                    aria-selected={tab === t.id}
                    className={`side-tab ${tab === t.id ? "active" : ""}`}
                    title={t.hint}
                    onClick={() => activateTab(t.id)}
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
                    {tabVisited("design") && (
                      <Suspense fallback={<PanelLoader label="设计工具" />}>
                        <StrategyDesignerPanel />
                        <UserPythonPanel />
                        <JsStrategyPanel />
                      </Suspense>
                    )}
                  </TabGroup>
                  <TabGroup id="validate" tab={tab}>
                    {tabVisited("validate") && (
                      <Suspense fallback={<PanelLoader label="验证工具" />}>
                        <ExcitationPanel />
                        <ScorePanel />
                        <GripPanel />
                        <ComparePanel />
                      </Suspense>
                    )}
                  </TabGroup>
                  <TabGroup id="data" tab={tab}>
                    {tabVisited("data") && (
                      <Suspense fallback={<PanelLoader label="数据工具" />}>
                        <MeasurePanel />
                        <RecordingPanel />
                        <ScriptPanel />
                        <ChartPanel />
                      </Suspense>
                    )}
                  </TabGroup>
                </ErrorBoundary>
              </div>
            </aside>
          </main>
        )}

        <Suspense fallback={<PageLoader />}>
          {page === "agent" && <ErrorBoundary label="Agent 工作台"><AgentWorkspace /></ErrorBoundary>}
          {page === "script" && <ErrorBoundary label="脚本工况"><ScriptWorkspace /></ErrorBoundary>}
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
                  <Suspense fallback={<PanelLoader label="场景工具" />}>
                    <ScenarioPanel />
                    <TrajectoryPanel />
                    <DisturbancePanel />
                    <FaultPanel />
                  </Suspense>
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
