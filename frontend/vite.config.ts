import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import path from "node:path";

const backendHttp = process.env.SIM4WIS_BACKEND_HTTP ?? "http://127.0.0.1:8010";
const backendWs = process.env.SIM4WIS_BACKEND_WS ?? backendHttp.replace(/^http/, "ws");

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
  build: {
    // Route and sidebar lazy loading keep the default workbench chunk near 500 kB.
    // Canvas3D is a known opt-in Three.js chunk and should stay visible if it
    // grows beyond this threshold.
    chunkSizeWarningLimit: 650,
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
