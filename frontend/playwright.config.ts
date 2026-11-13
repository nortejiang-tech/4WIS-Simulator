import { defineConfig, devices } from "@playwright/test";

// The e2e backend normally binds 8010; when a dev server already holds it,
// point the suite at a free port instead of failing with "port already used".
const port = Number(process.env.PW_E2E_PORT ?? 8010);

export default defineConfig({
  testDir: "./tests/e2e",
  timeout: 45_000,
  expect: {
    timeout: 10_000,
  },
  fullyParallel: false,
  // Every test file drives the same live simulator singleton.
  workers: 1,
  retries: 0,
  reporter: "list",
  use: {
    baseURL: `http://127.0.0.1:${port}`,
    trace: "retain-on-failure",
    // Allow an already installed browser for offline/controlled QA hosts.
    launchOptions: process.env.PW_CHROMIUM_EXECUTABLE_PATH
      ? { executablePath: process.env.PW_CHROMIUM_EXECUTABLE_PATH } : {},
  },
  webServer: {
    command: "node tests/e2e/start-backend.mjs",
    url: `http://127.0.0.1:${port}/health`,
    reuseExistingServer: false,
    timeout: 30_000,
  },
  projects: [
    {
      name: "chromium",
      use: { ...devices["Desktop Chrome"] },
    },
  ],
});
