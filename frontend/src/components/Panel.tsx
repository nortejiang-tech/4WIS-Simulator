/**
 * Panel — shared collapsible sidebar panel shell with an inline help (ⓘ) popover.
 *
 * Replaces the ad-hoc `<div className="panel"><h2>…</h2>…</div>` pattern so every
 * module gets a consistent header: a collapse toggle, an optional live status
 * badge, and a help button that reveals usage instructions in place.
 */

import { type ReactNode, useState } from "react";

import "./Panel.css";
import "./PanelContent.css";

interface PanelProps {
  title: string;
  help?: ReactNode;          // usage instructions, shown when ⓘ is toggled
  badge?: ReactNode;         // optional status element rendered right of the title
  defaultOpen?: boolean;
  children: ReactNode;
}

export default function Panel({ title, help, badge, defaultOpen = true, children }: PanelProps) {
  const [open, setOpen] = useState(defaultOpen);
  const [showHelp, setShowHelp] = useState(false);

  return (
    <div className="panel">
      <div className="panel-head">
        <button
          className="panel-toggle"
          onClick={() => setOpen((o) => !o)}
          aria-expanded={open}
          title={open ? "收起" : "展开"}
        >
          <span className="panel-caret">{open ? "▾" : "▸"}</span>
          <span className="panel-name">{title}</span>
        </button>
        {badge}
        {help != null && (
          <button
            className={`panel-help-btn ${showHelp ? "on" : ""}`}
            onClick={() => setShowHelp((s) => !s)}
            title="使用说明"
            aria-label="使用说明"
          >
            {showHelp ? "✕" : "ⓘ"}
          </button>
        )}
      </div>
      {showHelp && help != null && <div className="panel-help">{help}</div>}
      {open && <div className="panel-body">{children}</div>}
    </div>
  );
}
