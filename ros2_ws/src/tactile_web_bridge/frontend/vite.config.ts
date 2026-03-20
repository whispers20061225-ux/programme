import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

const runtimeEnv =
  (globalThis as { process?: { env?: Record<string, string | undefined> } }).process?.env ?? {};

const devHost = runtimeEnv.VITE_DEV_HOST || "127.0.0.1";
const devPort = Number(runtimeEnv.VITE_DEV_PORT || 5173);
const backendHttpTarget = runtimeEnv.VITE_BACKEND_HTTP_TARGET || "http://127.0.0.1:8765";
const backendWsTarget = runtimeEnv.VITE_BACKEND_WS_TARGET || "ws://127.0.0.1:8765";

export default defineConfig({
  plugins: [react()],
  server: {
    host: devHost,
    port: devPort,
    proxy: {
      "/api": {
        target: backendHttpTarget,
        changeOrigin: true,
      },
      "/ws": {
        target: backendWsTarget,
        changeOrigin: true,
        rewriteWsOrigin: true,
        ws: true,
      },
    },
  },
});
