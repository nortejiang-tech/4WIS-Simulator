import { useEffect, useState } from "react";

import Viewport from "@/components/Viewport";
import ControlPanel from "@/components/ControlPanel";
import ChartPanel from "@/components/ChartPanel";
import KeyboardInput from "@/components/KeyboardInput";
import ParamsPanel from "@/components/ParamsPanel";
import ProjectPanel from "@/components/ProjectPanel";
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
import LoadAnalysisPage from "@/components/LoadAnalysisPage";
import ModelTheoryPage from "@/components/ModelTheoryPage";
import QuickStartCard from "@/components/QuickStartCard";
import ErrorBoundary from "@/components/ErrorBoundary";
import Toasts from "@/components/Toasts";
import { connectSimSocket, fetchPath, fetchScenario } from "@/api/ws";
import { fetchJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";

// Sidebar tab groups. All panels stay mounted (so timers / WS subscriptions in
// ExcitationPanel / ScorePanel / MeasurePanel keep running across tab switches);
// inactive groups are hidden via CSS, not unmounted.
const TABS = [
  { id: "drive", label: "驾驶", hint: "策略 / 模型 / 油门 / 路面 / 仿真控制" },
  { id: "design", label: "设计", hint: "策略设计器 / 轨迹 / Python·JS 策略" },
  { id: "validate", label: "验证", hint: "开环激励 / 评分 / A/B 对比" },
  { id: "scene", label: "场景", hint: "车辆悬架参数 / 扰动 / 故障 / 项目" },
  { id: "data", label: "数据", hint: "测量 / 录制 / 脚本 / 实时曲线" },
] as const;
type TabId = (typeof TABS)[number]["id"];
type PageId = "sim" | "load" | "model";

// Keeps children mounted; uses display:contents when active so panels flow into
// the sidebar's flex column, display:none when inactive (state preserved).
function TabGroup({ id, tab, children }: { id: TabId; tab: TabId; children: React.ReactNode }) {
  return <div style={{ display: tab === id ? "contents" : "none" }}>{children}</div>;
}

const QUICKSTART_KEY = "4wis_quickstart_dismissed";

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
  const [tab, setTab] = useState<TabId>("drive");
  const [page, setPage] = useState<PageId>("sim");
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
        <nav className="page-tabs" aria-label="页面">
          <button className={page === "sim" ? "active" : ""} onClick={() => setPage("sim")}>
            仿真工作台
          </button>
          <button className={page === "load" ? "active" : ""} onClick={() => setPage("load")}>
            负载特性
          </button>
          <button className={page === "model" ? "active" : ""} onClick={() => setPage("model")}>
            数学模型
          </button>
        </nav>
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
        {page === "sim" && (
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

      {page === "sim" ? (
        <main className="app-main">
          <section className="viewport-pane">
            <ErrorBoundary label="视图">
              <Viewport />
            </ErrorBoundary>
            {quickStart && (
              <QuickStartCard
                onClose={dismissQuickStart}
                onGoTab={(t) => setTab(t as TabId)}
                onGoPage={(p) => setPage(p as PageId)}
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
                  <TrajectoryPanel />
                  <UserPythonPanel />
                  <JsStrategyPanel />
                </TabGroup>
                <TabGroup id="validate" tab={tab}>
                  <ExcitationPanel />
                  <ScorePanel />
                  <ComparePanel />
                </TabGroup>
                <TabGroup id="scene" tab={tab}>
                  <ScenarioPanel />
                  <ParamsPanel />
                  <DisturbancePanel />
                  <FaultPanel />
                  <ProjectPanel />
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
      ) : page === "load" ? (
        <ErrorBoundary label="负载特性">
          <LoadAnalysisPage />
        </ErrorBoundary>
      ) : (
        <ErrorBoundary label="数学模型">
          <ModelTheoryPage />
        </ErrorBoundary>
      )}

      {/* Listens to window-level keyboard events and pushes driver input */}
      <KeyboardInput />
      <Toasts />
    </div>
  );
}
