/**
 * ErrorBoundary — keeps one crashing region (e.g. the 3D canvas) from
 * taking down the whole app. Wrap viewport and sidebar separately.
 */

import { Component, type ReactNode } from "react";

interface Props {
  label: string;
  children: ReactNode;
}

interface State {
  error: Error | null;
}

export default class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null };

  static getDerivedStateFromError(error: Error): State {
    return { error };
  }

  componentDidCatch(error: Error) {
    console.error(`[ErrorBoundary:${this.props.label}]`, error);
  }

  render() {
    if (this.state.error) {
      return (
        <div style={{ padding: 16, color: "var(--text)" }}>
          <div style={{ fontWeight: 600, marginBottom: 6 }}>
            {this.props.label} 渲染出错
          </div>
          <div className="panel-small panel-mono" style={{ color: "var(--bad, #f87171)", marginBottom: 8 }}>
            {String(this.state.error?.message ?? this.state.error)}
          </div>
          <button onClick={() => this.setState({ error: null })}>重试</button>
        </div>
      );
    }
    return this.props.children;
  }
}
