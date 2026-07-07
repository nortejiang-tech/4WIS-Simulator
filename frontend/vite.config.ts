import { defineConfig, type Plugin } from "vite";
import react from "@vitejs/plugin-react";
import path from "node:path";

const backendHttp = process.env.SIM4WIS_BACKEND_HTTP ?? "http://127.0.0.1:8010";
const backendWs = process.env.SIM4WIS_BACKEND_WS ?? backendHttp.replace(/^http/, "ws");
const mainEntryChunkBudgetBytes = 500 * 1000;
const threeCoreChunkBudgetBytes = 700 * 1000;

function formatKb(bytes: number): string {
  return `${(bytes / 1000).toFixed(2)} kB`;
}

function chunkBudgetPlugin(): Plugin {
  return {
    name: "sim4wis-chunk-budgets",
    generateBundle(_, bundle) {
      for (const output of Object.values(bundle)) {
        if (output.type !== "chunk") continue;
        const size = Buffer.byteLength(output.code, "utf8");
        if (output.isEntry && size > mainEntryChunkBudgetBytes) {
          this.error(
            `main entry chunk ${output.fileName} is ${formatKb(size)}, above ${formatKb(mainEntryChunkBudgetBytes)}`,
          );
        }
        if (output.fileName.includes("vendor-three-core") && size > threeCoreChunkBudgetBytes) {
          this.error(
            `Three.js core chunk ${output.fileName} is ${formatKb(size)}, above ${formatKb(threeCoreChunkBudgetBytes)}`,
          );
        }
      }
    },
  };
}

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react(), chunkBudgetPlugin()],
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
  build: {
    // Route and sidebar lazy loading keep the default workbench chunk well under 700 kB.
    // Three.js stays opt-in via the lazy Canvas3D route; split its vendor stack
    // so the budget reflects the measured 3D vendor payload, not first-screen code.
    chunkSizeWarningLimit: 700,
    rollupOptions: {
      output: {
        manualChunks(id) {
          if (!id.includes("node_modules")) return undefined;
          if (id.includes("/node_modules/three/")) return "vendor-three-core";
          if (
            id.includes("/node_modules/@react-three/") ||
            id.includes("/node_modules/three-stdlib/") ||
            id.includes("/node_modules/@react-spring/three/")
          ) {
            return "vendor-three-react";
          }
          return undefined;
        },
      },
    },
  },
  server: {
    port: 5173,
    proxy: {
      // REST proxy — frontend code uses /api/* relative paths, dev server
      // forwards them to the FastAPI backend.
      "/api": {
        target: backendHttp,
        changeOrigin: true,
      },
      // WebSocket proxy.
      "/ws": {
        target: backendWs,
        ws: true,
        changeOrigin: true,
      },
    },
  },
});
