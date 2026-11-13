"""Run the full browser suite against a freshly extracted macOS release ZIP."""

from pathlib import Path
import argparse, json, os, shlex, subprocess, sys, tempfile, zipfile

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument("--runtime", dest="route", choices=["native", "cpp"], required=True)
p.add_argument("--archive", type=Path, required=True)
p.add_argument("--browser", default=os.environ.get("PW_CHROMIUM_EXECUTABLE_PATH"))
args = p.parse_args()
repo = ROOT
if sys.platform != "darwin":
    p.error(
        "this extracted-browser helper currently targets macOS; Windows has a native CI workflow"
    )
archive = args.archive.resolve()
with tempfile.TemporaryDirectory(prefix="4wis browser 解压 ") as tmp:
    temp = Path(tmp)
    with zipfile.ZipFile(archive) as z:
        z.extractall(temp)
        for info in z.infolist():
            mode = (info.external_attr >> 16) & 0o777
            if mode and not info.is_dir():
                (temp / info.filename).chmod(mode)
    root = next(x for x in temp.iterdir() if x.is_dir())
    port = 18081 if args.route == "native" else 18082
    env = os.environ.copy()
    for k in list(env):
        if k.startswith("SIM4WIS_") or k.startswith("PW_") or k == "PYTHONPATH":
            env.pop(k)
    env.update(
        SIM4WIS_DATA_DIR=str(root),
        SIM4WIS_RUNS_DIR=str(root / "runs"),
        SIM4WIS_EXPERIMENTS_DIR=str(root / "experiments"),
        PYTHONUTF8="1",
    )
    if args.route == "native":
        binary = root / "runtime/bin/python3"
        env["PYTHONPATH"] = os.pathsep.join(
            [str(root / "app/src"), str(root / "vendor")]
        )
        env["SIM4WIS_DIST_DIR"] = str(root / "app/dist")
        env["SIM4WIS_ENGINE"] = "native"
        command = [
            str(binary),
            "-m",
            "uvicorn",
            "sim4wis.main:app",
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ]
    else:
        binary = root / "app/sim4wis-cpp"
        env["SIM4WIS_DIST_DIR"] = str(root / "app/web")
        command = [str(binary), "--root", str(root), "--port", str(port)]
    driver = temp / "server.py"
    overrides = {
        k: v
        for k, v in env.items()
        if k.startswith("SIM4WIS_") or k in ("PYTHONPATH", "PYTHONUTF8")
    }
    driver.write_text(
        "import os\nos.chdir("
        + repr(str(root))
        + ')\nenv=os.environ.copy()\nfor k in list(env):\n if k.startswith("SIM4WIS_") or k.startswith("PW_") or k=="PYTHONPATH":env.pop(k)\nenv.update('
        + repr(overrides)
        + ")\nos.execvpe("
        + repr(str(binary))
        + ","
        + repr(command)
        + ",env)\n"
    )
    config = temp / "playwright.config.cjs"
    config.write_text(
        "const {defineConfig,devices}=require("
        + json.dumps(str(ROOT / "frontend/node_modules/@playwright/test"))
        + ");module.exports=defineConfig("
        + json.dumps(
            {
                "testDir": str(repo / "frontend/tests/e2e"),
                "outputDir": "/private/tmp/4wis-packaged-"
                + args.route
                + "-test-results",
                "timeout": 45000,
                "expect": {"timeout": 10000},
                "fullyParallel": False,
                "workers": 1,
                "retries": 0,
                "reporter": "list",
                "use": {
                    "baseURL": f"http://127.0.0.1:{port}",
                    "trace": "retain-on-failure",
                    "launchOptions": (
                        {"executablePath": args.browser} if args.browser else {}
                    ),
                },
                "webServer": {
                    "command": shlex.join([sys.executable, str(driver)]),
                    "url": f"http://127.0.0.1:{port}/health",
                    "reuseExistingServer": False,
                    "timeout": 30000,
                },
                "projects": [
                    {
                        "name": "chromium",
                        "use": {
                            "browserName": "chromium",
                            "viewport": {"width": 1280, "height": 720},
                            "deviceScaleFactor": 1,
                            "isMobile": False,
                            "hasTouch": False,
                        },
                    }
                ],
            }
        )
        + ");\n"
    )
    result = subprocess.run(
        ["npx", "playwright", "test", "--config", str(config)],
        cwd=repo / "frontend",
        env=env,
    )
    sys.exit(result.returncode)
