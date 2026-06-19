// Shared inline-style constants for panel controls (previously copy-pasted
// per panel). Uses the app-wide CSS variables so the theme applies.

import type { CSSProperties } from "react";

export const baseInput: CSSProperties = {
  background: "var(--bg-2)",
  border: "1px solid var(--border)",
  color: "var(--text)",
  borderRadius: 6,
  padding: "6px 8px",
  fontSize: 12,
};

export const selectStyle: CSSProperties = { ...baseInput, flex: 1 };

export const inputStyle: CSSProperties = {
  ...baseInput,
  fontFamily: "ui-monospace, SFMono-Regular, Menlo, monospace",
};

export const numInput: CSSProperties = {
  ...inputStyle,
  width: 96,
  flex: "0 0 auto",
};

export const activeBtn: CSSProperties = { background: "var(--accent)", color: "#fff" };
