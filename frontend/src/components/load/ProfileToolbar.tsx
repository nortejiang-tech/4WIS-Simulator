import { useState } from "react";

import { inputStyle, selectStyle } from "@/ui/styles";
import type { LoadRow, ProfileListItem } from "./types";

interface Props {
  profiles: ProfileListItem[];
  selected: string;
  onSelect: (name: string) => void;
  onLoad: () => void;
  onApply: () => void;
  onSave: (name: string) => void;
  onExport: () => void;
  busy: boolean;
  rows: LoadRow[];
}

export function ProfileToolbar({
  profiles, selected, onSelect, onLoad, onApply, onSave, onExport, busy, rows,
}: Props) {
  const [saveName, setSaveName] = useState("");
  return (
    <section className="load-toolbar">
      <div className="load-toolbar-group">
        <label>车型</label>
        <select value={selected} onChange={(e) => onSelect(e.target.value)} style={selectStyle}>
          {profiles.map((p) => <option key={p.name} value={p.name}>{p.label}</option>)}
        </select>
        <button onClick={onLoad} disabled={busy}>载入</button>
        <button onClick={onApply} disabled={busy}>应用到仿真</button>
      </div>
      <div className="load-toolbar-group">
        <input
          value={saveName}
          onChange={(e) => setSaveName(e.target.value)}
          placeholder="新车型名"
          style={{ ...inputStyle, width: 150 }}
        />
        <button
          onClick={() => {
            const name = saveName.trim();
            if (!name) return;
            onSave(name);
            setSaveName("");
          }}
          disabled={busy || !saveName.trim()}
        >保存车型</button>
        <button onClick={onExport} disabled={rows.length === 0}>导出 CSV</button>
      </div>
    </section>
  );
}
