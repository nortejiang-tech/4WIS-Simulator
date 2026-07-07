/**
 * Toasts — transient notifications (errors / confirmations) pushed via
 * useSimStore.pushToast. Rendered fixed at the bottom-centre of the app.
 */

import { useSimStore } from "@/store/sim";

export default function Toasts() {
  const toasts = useSimStore((s) => s.toasts);
  const dismiss = useSimStore((s) => s.dismissToast);
  if (toasts.length === 0) return null;

  return (
    <div
      aria-live="polite"
      data-testid="toast-stack"
      style={{
        position: "fixed", bottom: 16, left: "50%", transform: "translateX(-50%)",
        display: "flex", flexDirection: "column", gap: 6, zIndex: 1000,
        maxWidth: "70vw",
      }}
    >
      {toasts.map((t) => (
        <div
          key={t.id}
          role={t.kind === "error" ? "alert" : "status"}
          data-testid={`toast-${t.kind}`}
          onClick={() => dismiss(t.id)}
          style={{
            padding: "8px 14px", borderRadius: 8, fontSize: 13, cursor: "pointer",
            color: "#f8fafc",
            background: t.kind === "error" ? "rgba(153,27,27,0.95)" : "rgba(30,64,175,0.95)",
            border: `1px solid ${t.kind === "error" ? "#ef4444" : "#3b82f6"}`,
            boxShadow: "0 4px 16px rgba(0,0,0,0.35)",
          }}
        >
          {t.text}
        </div>
      ))}
    </div>
  );
}
