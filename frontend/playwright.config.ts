import { defineConfig, devices } from "@playwright/test";

export default defineConfig({
  testDir: "./tests/e2e",
  timeout: 45_000,
  expect: {
    timeout: 10_000,
  },
  fullyParallel: false,
  retries: 0,
  reporter: "list",
  use: {
    baseURL: "http://127.0.0.1:8010",
    trace: "retain-on-failure",
  },
  webServer: {
    command: [
      "rm -rf test-results/e2e-data",
      "mkdir -p test-results/e2e-data/experiments test-results/e2e-data/runs",
      [
        "SIM4WIS_EXPERIMENTS_DIR=test-results/e2e-data/experiments",
        "SIM4WIS_RUNS_DIR=test-results/e2e-data/runs",
        "../backend/.venv/bin/python -m uvicorn sim4wis.main:app --host 127.0.0.1 --port 8010",
      ].join(" "),
    ].join(" && "),
    url: "http://127.0.0.1:8010/health",
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
