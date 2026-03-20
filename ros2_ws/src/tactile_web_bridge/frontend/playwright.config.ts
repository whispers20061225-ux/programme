import { defineConfig, devices } from "@playwright/test";

const gatewayPort = Number(process.env.PLAYWRIGHT_GATEWAY_PORT || 8876);
const appPort = Number(process.env.PLAYWRIGHT_APP_PORT || 55173);
const gatewayBase = `http://127.0.0.1:${gatewayPort}`;
const appBase = `http://127.0.0.1:${appPort}`;

process.env.PLAYWRIGHT_GATEWAY_BASE = gatewayBase;

export default defineConfig({
  testDir: "./tests/e2e",
  fullyParallel: false,
  workers: 1,
  timeout: 30_000,
  reporter: [
    ["list"],
    ["html", { open: "never", outputFolder: "playwright-report" }],
  ],
  use: {
    baseURL: appBase,
    trace: "on-first-retry",
    screenshot: "only-on-failure",
    video: "retain-on-failure",
  },
  webServer: [
    {
      command: `MOCK_GATEWAY_PORT=${gatewayPort} npm run test:mock-gateway`,
      url: `${gatewayBase}/api/bootstrap`,
      timeout: 120_000,
      reuseExistingServer: false,
    },
    {
      command: `VITE_DEV_HOST=127.0.0.1 VITE_DEV_PORT=${appPort} VITE_BACKEND_HTTP_TARGET=${gatewayBase} VITE_BACKEND_WS_TARGET=ws://127.0.0.1:${gatewayPort} npm run dev -- --host 127.0.0.1 --port ${appPort}`,
      url: `${appBase}/control`,
      timeout: 120_000,
      reuseExistingServer: false,
    },
  ],
  projects: [
    {
      name: "chromium",
      use: { ...devices["Desktop Chrome"] },
    },
  ],
});
