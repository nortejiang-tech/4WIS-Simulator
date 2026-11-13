// Cross-platform test-only backend launcher. Data remains in test-results.
import { mkdirSync, rmSync } from "node:fs";
import { spawn } from "node:child_process";

const root = "test-results/e2e-data";
rmSync(root, { recursive: true, force: true });
mkdirSync(`${root}/experiments`, { recursive: true });
mkdirSync(`${root}/runs`, { recursive: true });
const python = process.env.PW_PYTHON_EXECUTABLE ??
  (process.platform === "win32" ? "../backend/.venv/Scripts/python.exe" : "../backend/.venv/bin/python");
const executable = process.env.PW_CPP_EXECUTABLE ?? python;
const args = process.env.PW_CPP_EXECUTABLE
  ? ["--serve", "--root", "..", "--port", process.env.PW_E2E_PORT ?? "8010"]
  : ["-m", "uvicorn", "sim4wis.main:app", "--host", "127.0.0.1", "--port", process.env.PW_E2E_PORT ?? "8010"];
const child = spawn(executable, args, {
  stdio: "inherit",
  env: { ...process.env, SIM4WIS_EXPERIMENTS_DIR: `${root}/experiments`, SIM4WIS_RUNS_DIR: `${root}/runs` },
});
child.on("error", (error) => { console.error(error); process.exit(1); });
child.on("exit", (code) => process.exit(code ?? 1));
for (const signal of ["SIGINT", "SIGTERM"]) process.on(signal, () => child.kill(signal));
