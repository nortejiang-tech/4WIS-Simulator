import { expect, test, type Locator, type Page, type TestInfo } from "@playwright/test";

async function dragBy(locator: Locator, dx: number, dy: number) {
  const box = await locator.boundingBox();
  if (!box) throw new Error("target is not visible for dragging");
  const x = box.x + box.width / 2;
  const y = box.y + box.height / 2;
  await locator.page().mouse.move(x, y);
  await locator.page().mouse.down();
  await locator.page().mouse.move(x + dx, y + dy, { steps: 6 });
  await locator.page().mouse.up();
}

async function hoverInside(locator: Locator, xRatio: number, yRatio = 0.5) {
  const box = await locator.boundingBox();
  if (!box) throw new Error("target is not visible for hovering");
  await locator.hover({ position: { x: box.width * xRatio, y: box.height * yRatio } });
}

async function setRangeValue(locator: Locator, value: string) {
  await locator.evaluate((el, nextValue) => {
    const input = el as HTMLInputElement;
    const setter = Object.getOwnPropertyDescriptor(window.HTMLInputElement.prototype, "value")?.set;
    setter?.call(input, nextValue);
    input.dispatchEvent(new Event("input", { bubbles: true }));
    input.dispatchEvent(new Event("change", { bubbles: true }));
  }, value);
}

async function expectCanvasHasDrawnPixels(locator: Locator) {
  const stats = await locator.evaluate((el) => {
    const canvas = el as HTMLCanvasElement;
    const ctx = canvas.getContext("2d");
    if (!ctx) return { width: canvas.width, height: canvas.height, ink: 0 };
    const image = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
    let ink = 0;
    for (let i = 3; i < image.length; i += 4) {
      if (image[i] > 0) ink += 1;
      if (ink > 200) break;
    }
    return { width: canvas.width, height: canvas.height, ink };
  });
  expect(stats.width).toBeGreaterThan(100);
  expect(stats.height).toBeGreaterThan(100);
  expect(stats.ink).toBeGreaterThan(200);
}

async function attachPageScreenshot(page: Page, testInfo: TestInfo, name: string) {
  const screenshot = await page.screenshot({ fullPage: true });
  expect(screenshot.length).toBeGreaterThan(10_000);
  await testInfo.attach(name, { body: screenshot, contentType: "image/png" });
}

async function chartCursorX(locator: Locator) {
  return locator.evaluate((el) => Number((el as HTMLElement).dataset.chartCursorX));
}

async function chartXSpan(locator: Locator) {
  return locator.evaluate((el) => Number((el as HTMLElement).dataset.chartXSpan));
}

async function dragChartSelection(locator: Locator, fromRatio: number, toRatio: number) {
  const box = await locator.boundingBox();
  if (!box) throw new Error("chart overlay is not visible for drag zoom");
  const y = box.y + box.height * 0.45;
  await locator.page().mouse.move(box.x + box.width * fromRatio, y);
  await locator.page().mouse.down();
  await locator.page().mouse.move(box.x + box.width * toRatio, y, { steps: 8 });
  await locator.page().mouse.up();
}

test("workflow rail pages render from the production app shell", async ({ page }, testInfo) => {
  await page.goto("/");

  await expect(page.locator(".brand")).toHaveText("4WIS");
  await expect(page.locator(".title")).toHaveText("Simulator");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await expect(rail).toBeVisible();
  await expect(rail.getByRole("button", { name: /运行/ })).toHaveClass(/active/);
  await attachPageScreenshot(page, testInfo, "workflow-run");

  await rail.getByRole("button", { name: /试验/ }).click();
  await expect(page.getByText("运行矩阵")).toBeVisible();
  await attachPageScreenshot(page, testInfo, "workflow-experiment");

  await rail.getByRole("button", { name: /分析/ }).click();
  await expect(page.getByText("还没有 run — 到「试验」页跑一个批量")).toBeVisible();
  await expect(page.getByText(/从左侧选择 1.*6 个 run/)).toBeVisible();
  await attachPageScreenshot(page, testInfo, "workflow-analysis-empty");

  await rail.getByRole("button", { name: /车辆/ }).click();
  await expect(page.getByText("车辆 / 悬架参数").first()).toBeVisible();
  await expect(page.getByRole("img", { name: "整车俯视参数示意" })).toBeVisible();
  await attachPageScreenshot(page, testInfo, "workflow-vehicle");

  await rail.getByRole("button", { name: /场景/ }).click();
  await expect(page.getByText("场景").first()).toBeVisible();
  await expect(page.getByText("轨迹 / 路径").first()).toBeVisible();
  await attachPageScreenshot(page, testInfo, "workflow-scenario");

  await rail.getByRole("button", { name: /负载/ }).click();
  await expect(page.getByText("实时四轮负载")).toBeVisible();
  await attachPageScreenshot(page, testInfo, "workflow-load");

  await rail.getByRole("button", { name: /原理/ }).click();
  await expect(page.getByText("4WIS foundation model")).toBeVisible();
  await attachPageScreenshot(page, testInfo, "workflow-theory");
});

test("model theory demo failure stays visible without blanking the page", async ({ page }, testInfo) => {
  await page.route("**/api/model/demo/tire-curve", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic tire demo failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /原理/ }).click();

  await expect(page.getByText("4WIS foundation model")).toBeVisible();
  const alert = page.getByText("轮胎演示计算失败：synthetic tire demo failure").first();
  await alert.scrollIntoViewIfNeeded();
  await expect(alert).toBeVisible();
  await expect(page.getByText("轮胎 Fy 随侧偏角 α（峰值 = μ·Fz）")).toBeVisible();
  await expect(page.getByText("等待计算").first()).toBeVisible();

  await attachPageScreenshot(page, testInfo, "workflow-model-demo-error");
});

test("experiment run can be handed to analysis with chart controls and screenshot evidence", async ({ page }, testInfo) => {
  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });

  await rail.getByRole("button", { name: /试验/ }).click();
  await expect(page.getByText("运行矩阵")).toBeVisible();

  await page.getByRole("button", { name: /运行（1 runs）/ }).click();
  await expect(page.getByText(/完成 1 runs/)).toBeVisible({ timeout: 30_000 });
  await expect(page.getByRole("table").filter({ hasText: "横摆峰值°/s" })).toBeVisible();

  await page.getByRole("button", { name: /去分析页对比这些 runs/ }).click();
  await expect(rail.getByRole("button", { name: /分析/ })).toHaveClass(/active/);
  await expect(page.getByText("KPI 对比")).toBeVisible();
  await expect(page.getByText("横摆角速度峰值 °/s")).toBeVisible();
  await expect(page.getByText("通道叠图")).toBeVisible();
  await expect(page.locator(".uplot").first()).toBeVisible();

  await expectCanvasHasDrawnPixels(page.getByTestId("analysis-chart-vx").locator("canvas").first());
  await expectCanvasHasDrawnPixels(page.getByTestId("analysis-trajectory-canvas"));

  await page.getByTestId("analysis-chip-driver_steering").click();
  await expect(page.getByTestId("analysis-chart-driver_steering")).toBeVisible();

  await page.getByTestId("analysis-chip-vx").click();
  await expect(page.getByTestId("analysis-chart-vx")).toHaveCount(0);

  await page.getByLabel("分析通道选择").selectOption("rack_force_fl");
  await page.getByRole("button", { name: /加图/ }).click();
  await expect(page.getByTestId("analysis-chart-rack_force_fl")).toBeVisible();
  const rackCanvas = page.getByTestId("analysis-chart-rack_force_fl").locator("canvas").first();
  await expectCanvasHasDrawnPixels(rackCanvas);

  const rackChartBody = page.getByTestId("analysis-chart-rack_force_fl").locator(".wf-chart-body").first();
  const rackChartOverlay = page.getByTestId("analysis-chart-rack_force_fl").locator(".u-over").first();
  await hoverInside(page.getByTestId("analysis-chart-rack_force_fl").locator(".u-over").first(), 0.5);
  await expect.poll(async () => chartCursorX(rackChartBody)).toBeGreaterThan(0);

  const initialSpan = await chartXSpan(rackChartBody);
  expect(initialSpan).toBeGreaterThan(1);
  await dragChartSelection(rackChartOverlay, 0.2, 0.7);
  await expect.poll(async () => chartXSpan(rackChartBody)).toBeLessThan(initialSpan * 0.75);

  await rackChartOverlay.dblclick({ position: { x: 12, y: 12 } });
  await expect.poll(async () => chartXSpan(rackChartBody)).toBeGreaterThan(initialSpan * 0.9);

  const downloadPromise = page.waitForEvent("download");
  await page.getByTestId("analysis-chart-png-rack_force_fl").click();
  const download = await downloadPromise;
  expect(download.suggestedFilename()).toBe("rack_force_fl.png");

  const screenshot = await page.getByTestId("analysis-workbench").screenshot();
  expect(screenshot.length).toBeGreaterThan(10_000);
  await testInfo.attach("analysis-workbench", { body: screenshot, contentType: "image/png" });
});

test("analysis data load failure surfaces an error state with screenshot evidence", async ({ page }, testInfo) => {
  const runId = "synthetic-run-for-error-state";
  await page.route("**/api/runs", async (route) => {
    await route.fulfill({
      contentType: "application/json",
      body: JSON.stringify({
        runs: [{
          run_id: runId,
          label: "synthetic error run",
          created_at: "2026-07-07T00:00:00",
          duration_s: 8,
          n_samples: 400,
          strategy: "ideal_ackermann",
          model_type: "kinematic",
          experiment_name: "error_state_fixture",
          kpis: { yaw_rate_peak_dps: 1.23 },
          job_id: null,
        }],
      }),
    });
  });
  await page.route(`**/api/runs/${runId}/data**`, async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic channel load failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /分析/ }).click();

  await expect(page.getByText("Run 库（1）")).toBeVisible();
  await page.getByText("synthetic error run").click();

  const alert = page.getByRole("alert");
  await expect(alert).toContainText("读取 run 数据失败：synthetic channel load failure");
  await expect(page.getByText("KPI 对比")).toBeVisible();
  await expect(page.getByTestId("analysis-kpi-table")).toContainText("横摆角速度峰值 °/s");
  await expect(page.getByTestId("analysis-kpi-table")).toContainText("1.23");

  await attachPageScreenshot(page, testInfo, "workflow-analysis-data-error");
});

test("analysis run list failure does not masquerade as an empty library", async ({ page }, testInfo) => {
  await page.route("**/api/runs", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic run list failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /分析/ }).click();

  await expect(page.getByText("Run 库（0）")).toBeVisible();
  await expect(page.getByText("读取 run 列表失败：synthetic run list failure").first()).toBeVisible();
  await expect(page.getByText("还没有 run — 到「试验」页跑一个批量")).toHaveCount(0);
  await expect(page.getByRole("button", { name: "刷新" })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-analysis-run-list-error");
});

test("experiment library failure does not masquerade as an empty library", async ({ page }, testInfo) => {
  await page.route("**/api/experiments", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic experiment library failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /试验/ }).click();

  const library = page
    .locator("aside.wf-col")
    .filter({ has: page.locator(".wf-col-head", { hasText: "实验库" }) });
  await expect(library).toBeVisible();
  await expect(library).toContainText("读取实验库失败：synthetic experiment library failure");
  await expect(library.getByText("暂无已保存实验")).toHaveCount(0);
  await expect(library.getByRole("button", { name: "刷新" })).toBeEnabled();
  await expect(library.getByRole("button", { name: "＋ 新建" })).toBeEnabled();
  await expect(page.getByText("运行矩阵")).toBeVisible();

  await attachPageScreenshot(page, testInfo, "workflow-experiment-library-error");
});

test("experiment maneuver template failure stays visible without blocking editing", async ({ page }, testInfo) => {
  await page.route("**/api/maneuver-templates", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic maneuver template failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /试验/ }).click();

  await expect(page.getByText("运行矩阵")).toBeVisible();
  await expect(page.getByText("读取机动模板失败：synthetic maneuver template failure").first()).toBeVisible();
  const pathSelect = page.getByLabel("参考路径");
  await expect(pathSelect).toBeEnabled();
  await expect(pathSelect.locator("option")).toHaveCount(1);
  await expect(page.getByRole("button", { name: /运行（1 runs）/ })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-experiment-maneuver-template-error");
});

test("experiment batch start failure surfaces an error state with screenshot evidence", async ({ page }, testInfo) => {
  await page.route("**/api/batch", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 422,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic batch validation failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /试验/ }).click();

  await expect(page.getByText("运行矩阵")).toBeVisible();
  await page.getByRole("button", { name: /运行（1 runs）/ }).click();

  const alert = page.getByRole("alert");
  await expect(alert).toContainText("启动失败：synthetic batch validation failure");
  await expect(page.getByText("运行矩阵")).toBeVisible();
  await expect(page.getByRole("button", { name: /运行（1 runs）/ })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-experiment-batch-error");
});

test("run control backend failures stay visible in the control panels", async ({ page }, testInfo) => {
  await page.route("**/api/model", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic model switch failure" }),
    });
  });
  await page.route("**/api/scene/mu", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic road mu failure" }),
    });
  });

  await page.goto("/");

  const modelPanel = page.locator(".panel").filter({ hasText: "动力学模型" });
  await expect(modelPanel).toBeVisible();
  await modelPanel.getByRole("button", { name: "多体(14DOF)" }).click();
  await expect(page.getByRole("alert")).toContainText("模型切换失败：synthetic model switch failure");
  await expect(modelPanel.getByRole("button", { name: "多体(14DOF)" })).toBeEnabled();

  const frictionPanel = page.locator(".panel").filter({ hasText: "路面摩擦" });
  await expect(frictionPanel).toBeVisible();
  await frictionPanel.locator("select").selectOption("雪面");
  await expect(page.getByRole("alert").last()).toContainText("路面摩擦设置失败：synthetic road mu failure");
  await expect(frictionPanel.locator("select")).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-run-control-error");
});

test("user python status failure stays visible in the design panel", async ({ page }, testInfo) => {
  await page.route("**/api/user_python/status", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic user python status failure" }),
    });
  });

  await page.goto("/");
  await page.getByRole("tab", { name: "设计" }).click();

  const panel = page.locator(".panel").filter({ hasText: "Python 策略插件" });
  await expect(panel).toBeVisible();
  await expect(panel).toContainText("状态未知");
  await expect(panel.getByRole("alert")).toContainText(
    "读取 Python 策略状态失败：synthetic user python status failure",
  );
  await expect(panel.getByText("文件不存在")).toHaveCount(0);

  await attachPageScreenshot(page, testInfo, "workflow-user-python-status-error");
});

test("vehicle geometry drag updates the shared parameter edit buffer", async ({ page }) => {
  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });

  await rail.getByRole("button", { name: /车辆/ }).click();
  await expect(page.getByRole("img", { name: "整车俯视参数示意" })).toBeVisible();

  const wheelbaseInput = page.getByLabel("轴距 L (mm)");
  await expect(wheelbaseInput).toBeVisible();
  const before = Number(await wheelbaseInput.inputValue());

  await dragBy(page.getByTestId("vg-handle-wheelbase"), 0, -36);

  await expect(page.getByText("有未应用的几何改动")).toBeVisible();
  await expect.poll(async () => Number(await wheelbaseInput.inputValue())).not.toBe(before);
  await expect(page.locator(".vg-dirtybar").getByRole("button", { name: "应用" })).toBeEnabled();
});

test("vehicle parameter rejection keeps edits visible with screenshot evidence", async ({ page }, testInfo) => {
  await page.route("**/api/params", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 422,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic parameter bounds failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /车辆/ }).click();
  await expect(page.getByRole("img", { name: "整车俯视参数示意" })).toBeVisible();

  const wheelbaseInput = page.getByLabel("轴距 L (mm)");
  const before = Number(await wheelbaseInput.inputValue());
  await wheelbaseInput.fill(String(before + 10));
  await expect(page.getByText("有未应用的几何改动")).toBeVisible();

  await page.getByRole("button", { name: "应用参数" }).click();

  const alert = page.getByRole("alert");
  await expect(alert).toContainText("参数被拒绝：synthetic parameter bounds failure");
  await expect(page.getByText("有未应用的几何改动")).toBeVisible();
  await expect(page.getByRole("button", { name: "应用参数" })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-vehicle-param-error");
});

test("vehicle project list failure is visible and disables loading", async ({ page }, testInfo) => {
  await page.route("**/api/projects", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic project list failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /车辆/ }).click();

  const projectPanel = page.locator(".panel").filter({ hasText: "项目（YAML）" });
  await expect(projectPanel).toBeVisible();
  await expect(projectPanel).toContainText("读取项目列表失败：synthetic project list failure");
  await expect(projectPanel.locator("select")).toHaveValue("");
  await expect(projectPanel.getByRole("button", { name: "加载" })).toBeDisabled();
  await expect(page.getByRole("img", { name: "整车俯视参数示意" })).toBeVisible();

  await attachPageScreenshot(page, testInfo, "workflow-vehicle-project-list-error");
});

test("vehicle project load failure stays local to the project panel", async ({ page }, testInfo) => {
  await page.route("**/api/projects", async (route) => {
    await route.fulfill({
      contentType: "application/json",
      body: JSON.stringify({ projects: ["synthetic_failure.yaml"] }),
    });
  });
  await page.route("**/api/projects/*/load", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic project load failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /车辆/ }).click();

  const projectPanel = page.locator(".panel").filter({ hasText: "项目（YAML）" });
  await expect(projectPanel).toBeVisible();
  await expect(projectPanel.locator("select")).toHaveValue("synthetic_failure.yaml");

  await projectPanel.getByRole("button", { name: "加载" }).click();
  await expect(projectPanel).toContainText("synthetic project load failure");
  await expect(projectPanel.getByRole("button", { name: "加载" })).toBeEnabled();
  await expect(page.getByRole("img", { name: "整车俯视参数示意" })).toBeVisible();

  await attachPageScreenshot(page, testInfo, "workflow-vehicle-project-load-error");
});

test("load analysis charts expose deeper explanation state with screenshot evidence", async ({ page }, testInfo) => {
  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /负载/ }).click();

  await expect(page.getByText("实时四轮负载")).toBeVisible();
  await expect(page.locator(".load-chart-panel").first()).toBeVisible();

  const firstChartCanvas = page.locator(".load-chart-host canvas").first();
  await expect(firstChartCanvas).toBeVisible({ timeout: 20_000 });
  await expectCanvasHasDrawnPixels(firstChartCanvas);

  await page.locator(".load-chart-help-btn").first().click();
  await expect(page.locator(".load-explanation-modal")).toBeVisible();
  await expect(page.locator(".load-explanation-modal h3")).not.toHaveText("");

  await attachPageScreenshot(page, testInfo, "workflow-load-explanation");
});

test("load analysis sweep failure surfaces an error and keeps controls usable", async ({ page }, testInfo) => {
  let sweepAttempts = 0;
  await page.route("**/api/load-analysis/sweep", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    sweepAttempts += 1;
    if (sweepAttempts > 1) {
      await route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify({ rows: [], body_coupling: "vehicle", summary: { warnings: [] } }),
      });
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic load sweep failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /负载/ }).click();

  await expect(page.getByText("实时四轮负载")).toBeVisible();
  await expect(page.getByText("负载扫图失败：synthetic load sweep failure").first()).toBeVisible();
  await expect(page.getByRole("button", { name: "重新计算" })).toBeEnabled();
  await expect(page.locator(".load-chart-empty").first()).toContainText("等待计算");

  await attachPageScreenshot(page, testInfo, "workflow-load-sweep-error");
});

test("load analysis profile list failure surfaces an error without blanking the page", async ({ page }, testInfo) => {
  await page.route("**/api/vehicle-profiles", async (route) => {
    if (route.request().method() === "GET") {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic profile list failure" }),
      });
      return;
    }
    await route.fallback();
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /负载/ }).click();

  await expect(page.getByText("实时四轮负载")).toBeVisible();
  await expect(page.getByText("读取车型库失败：synthetic profile list failure").first()).toBeVisible();
  await expect(page.getByRole("button", { name: "重新计算" })).toBeEnabled();
  await expect(page.getByRole("button", { name: "重置 LS9" })).toBeEnabled();
  await expect(page.locator(".load-chart-panel").first()).toBeVisible();

  await attachPageScreenshot(page, testInfo, "workflow-load-profile-list-error");
});

test("load analysis params failure leaves a visible empty state", async ({ page }, testInfo) => {
  await page.route("**/api/params", async (route) => {
    if (route.request().method() === "GET") {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic load params failure" }),
      });
      return;
    }
    await route.fallback();
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /负载/ }).click();

  await expect(page.getByText("实时四轮负载")).toBeVisible();
  await expect(page.getByText("读取参数失败：synthetic load params failure").first()).toBeVisible();
  await expect(page.locator(".load-param-pane")).toContainText("读取中...");
  await expect(page.getByRole("button", { name: "重新计算" })).toBeDisabled();
  await expect(page.getByRole("button", { name: "重置 LS9" })).toBeEnabled();
  await expect(page.locator(".load-chart-panel").first()).toBeVisible();

  await attachPageScreenshot(page, testInfo, "workflow-load-params-error");
});

test("load analysis profile action failures stay visible without blocking the toolbar", async ({ page }, testInfo) => {
  await page.route("**/api/vehicle-profiles/LS9", async (route) => {
    if (route.request().method() === "GET") {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic profile load failure" }),
      });
      return;
    }
    await route.fallback();
  });
  await page.route("**/api/vehicle-profiles/LS9/apply", async (route) => {
    if (route.request().method() === "POST") {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic profile apply failure" }),
      });
      return;
    }
    await route.fallback();
  });
  await page.route("**/api/vehicle-profiles/bad_profile", async (route) => {
    if (route.request().method() === "POST") {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic profile save failure" }),
      });
      return;
    }
    await route.fallback();
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /负载/ }).click();

  await expect(page.getByText("实时四轮负载")).toBeVisible();
  await expect(page.getByRole("button", { name: "重新计算" })).toBeEnabled();

  await page.getByRole("button", { name: "载入" }).click();
  await expect(page.getByText("载入车型失败：synthetic profile load failure").first()).toBeVisible();
  await expect(page.getByRole("button", { name: "载入" })).toBeEnabled();

  await page.getByRole("button", { name: "应用到仿真" }).click();
  await expect(page.getByText("应用车型失败：synthetic profile apply failure").first()).toBeVisible();
  await expect(page.getByRole("button", { name: "应用到仿真" })).toBeEnabled();

  await page.getByPlaceholder("新车型名").fill("bad_profile");
  await page.getByRole("button", { name: "保存车型" }).click();
  await expect(page.getByText("保存车型失败：synthetic profile save failure").first()).toBeVisible();
  await expect(page.getByRole("button", { name: "重新计算" })).toBeEnabled();
  await expect(page.locator(".load-chart-panel").first()).toBeVisible();

  await attachPageScreenshot(page, testInfo, "workflow-load-profile-action-errors");
});

test("load analysis sensitivity failure surfaces a toast without blanking load charts", async ({ page }, testInfo) => {
  await page.route("**/api/load-analysis/sensitivity", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic sensitivity failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /负载/ }).click();

  await expect(page.getByText("实时四轮负载")).toBeVisible();
  await expect(page.getByText(/δ_eq 敏感度/)).toBeVisible({ timeout: 20_000 });
  await expect(page.getByText("敏感度扫描失败：synthetic sensitivity failure").first()).toBeVisible();
  await expect(page.locator(".load-sensitivity").getByRole("button", { name: "扫描" })).toBeEnabled();

  const firstChartCanvas = page.locator(".load-chart-host canvas").first();
  await expect(firstChartCanvas).toBeVisible({ timeout: 20_000 });
  await expectCanvasHasDrawnPixels(firstChartCanvas);

  await attachPageScreenshot(page, testInfo, "workflow-load-sensitivity-error");
});

test("scenario path workflow generates, follows, and clears a reference path", async ({ page }, testInfo) => {
  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const trajectoryPanel = page.locator(".panel").filter({ hasText: "轨迹 / 路径" });
  await expect(trajectoryPanel).toBeVisible();
  await expect(trajectoryPanel).toContainText("无路径");

  await trajectoryPanel.locator("select").first().selectOption("double_lane_change");
  await trajectoryPanel.getByRole("button", { name: "生成" }).click();
  await expect(trajectoryPanel).toContainText(/当前路径: double_lane_change · \d+ 点 · \d+ 桩/);

  await trajectoryPanel.getByRole("button", { name: "跟踪此路径" }).click();
  await expect(page.getByLabel("当前状态摘要")).toContainText("follow_trajectory");

  await trajectoryPanel.getByRole("button", { name: "清除路径" }).click();
  await expect(trajectoryPanel).toContainText("无路径");

  await attachPageScreenshot(page, testInfo, "workflow-scenario-path");
});

test("path version refresh failure surfaces a toast without blocking scenario tools", async ({ page }, testInfo) => {
  let failPathRefresh = false;
  await page.route("**/api/path", async (route) => {
    if (route.request().method() === "GET" && failPathRefresh) {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic path refresh failure" }),
      });
      return;
    }
    await route.fallback();
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const trajectoryPanel = page.locator(".panel").filter({ hasText: "轨迹 / 路径" });
  await expect(trajectoryPanel).toBeVisible();

  await trajectoryPanel.locator("select").first().selectOption("double_lane_change");
  await trajectoryPanel.getByRole("button", { name: "生成" }).click();
  await expect(trajectoryPanel).toContainText(/当前路径: double_lane_change · \d+ 点 · \d+ 桩/);

  failPathRefresh = true;
  await trajectoryPanel.getByRole("button", { name: "清除路径" }).click();
  await expect(page.getByText("刷新参考路径失败：synthetic path refresh failure").first()).toBeVisible();
  await expect(trajectoryPanel.getByRole("button", { name: "生成" })).toBeEnabled();
  await expect(trajectoryPanel.getByRole("button", { name: "手动绘制" })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-path-refresh-error");
});

test("path template failure stays visible in the trajectory panel", async ({ page }, testInfo) => {
  await page.route("**/api/path/templates", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic path template failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const trajectoryPanel = page.locator(".panel").filter({ hasText: "轨迹 / 路径" });
  await expect(trajectoryPanel).toBeVisible();
  await expect(trajectoryPanel).toContainText("读取路径模板失败：synthetic path template failure");
  await expect(trajectoryPanel.locator("select")).toBeDisabled();
  await expect(trajectoryPanel.getByRole("button", { name: "生成" })).toBeDisabled();
  await expect(trajectoryPanel.getByRole("button", { name: "刷新模板" })).toBeEnabled();
  await expect(trajectoryPanel.getByRole("button", { name: "手动绘制" })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-path-template-error");
});

test("scenario list failure stays visible in the scenario panel", async ({ page }, testInfo) => {
  await page.route("**/api/scenarios", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic scenario list failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const scenarioPanel = page.locator(".panel").filter({ hasText: "场景路况" });
  await expect(scenarioPanel).toBeVisible();
  await expect(scenarioPanel).toContainText("读取场景列表失败：synthetic scenario list failure");
  await expect(scenarioPanel.locator(".strategy-buttons button")).toHaveCount(0);
  await expect(scenarioPanel.getByRole("button", { name: "刷新场景" })).toBeEnabled();
  await expect(scenarioPanel.getByRole("button", { name: "清除场景" })).toBeDisabled();

  await attachPageScreenshot(page, testInfo, "workflow-scenario-list-error");
});

test("scenario load and clear failures keep the scenario panel recoverable", async ({ page }, testInfo) => {
  let failLoad = true;
  await page.route("**/api/scenarios/*/load", async (route) => {
    if (route.request().method() !== "POST" || !failLoad) {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic scenario load failure" }),
    });
  });

  await page.route("**/api/scenario/clear", async (route) => {
    if (route.request().method() !== "POST") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic scenario clear failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const scenarioPanel = page.locator(".panel").filter({ hasText: "场景路况" });
  await expect(scenarioPanel).toBeVisible();

  await scenarioPanel.getByRole("button", { name: "广场" }).click();
  await expect(page.getByText("加载场景失败：synthetic scenario load failure").first()).toBeVisible();
  await expect(scenarioPanel).toContainText("当前：无（空网格）");
  await expect(scenarioPanel.getByRole("button", { name: "广场" })).toBeEnabled();

  failLoad = false;
  await scenarioPanel.getByRole("button", { name: "广场" }).click();
  await expect(scenarioPanel).toContainText("当前：广场");

  await scenarioPanel.getByRole("button", { name: "清除场景" }).click();
  await expect(page.getByText("清除场景失败：synthetic scenario clear failure").first()).toBeVisible();
  await expect(scenarioPanel).toContainText("当前：广场");
  await expect(scenarioPanel.getByRole("button", { name: "清除场景" })).toBeEnabled();
  await expect(scenarioPanel.getByRole("button", { name: "刷新场景" })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-scenario-load-clear-errors");
});

test("scenario version refresh failure surfaces a toast without blocking scenario tools", async ({ page }, testInfo) => {
  let failScenarioRefresh = false;
  await page.route("**/api/scenario", async (route) => {
    if (route.request().method() === "GET" && failScenarioRefresh) {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic scenario refresh failure" }),
      });
      return;
    }
    await route.fallback();
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const scenarioPanel = page.locator(".panel").filter({ hasText: "场景路况" });
  await expect(scenarioPanel).toBeVisible();
  await scenarioPanel.getByRole("button", { name: "广场" }).click();
  await expect(scenarioPanel).toContainText("当前：广场");

  failScenarioRefresh = true;
  await scenarioPanel.getByRole("button", { name: "清除场景" }).click();
  await expect(page.getByText("刷新场景几何失败：synthetic scenario refresh failure").first()).toBeVisible();
  await expect(scenarioPanel.getByRole("button", { name: "广场" })).toBeEnabled();
  await expect(scenarioPanel.getByRole("button", { name: "刷新场景" })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-scenario-refresh-error");
});

test("scenario disturbance editor places, edits, and clears a road disturbance", async ({ page, request }, testInfo) => {
  await request.post("/api/scene/clear");

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const disturbancePanel = page.locator(".panel").filter({ hasText: "路面扰动编辑" });
  await expect(disturbancePanel).toBeVisible();
  await expect(disturbancePanel).toContainText("无扰动");

  await disturbancePanel.locator("select").selectOption("speed_bump");
  await disturbancePanel.getByRole("button", { name: "放置" }).click();
  await expect(disturbancePanel).toContainText("放置中");

  const canvas = page.locator(".canvas-container canvas").first();
  await expect(canvas).toBeVisible();
  const box = await canvas.boundingBox();
  if (!box) throw new Error("2D canvas is not visible for disturbance placement");
  await page.mouse.click(box.x + box.width * 0.55, box.y + box.height * 0.48);

  await expect(disturbancePanel).toContainText("减速带");
  await expect(disturbancePanel).toContainText("编辑");

  const fields = disturbancePanel.locator('input[type="number"]');
  await expect(fields).toHaveCount(7);
  await fields.nth(5).fill("0.18");
  await fields.nth(6).fill("220000");
  await disturbancePanel.getByRole("button", { name: "应用修改" }).click();
  await expect(page.getByText(/已更新 speed_bump_/)).toBeVisible();

  await disturbancePanel.getByRole("button", { name: "清空全部" }).click();
  await expect(page.getByText("已清空 1 个扰动")).toBeVisible();
  await expect(disturbancePanel).toContainText("无扰动");

  await attachPageScreenshot(page, testInfo, "workflow-scenario-disturbance");
});

test("scenario fault injection panel adds, toggles, and clears faults", async ({ page, request }, testInfo) => {
  await request.delete("/api/faults");

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const faultPanel = page.locator(".panel").filter({ hasText: "故障注入" });
  await expect(faultPanel).toBeVisible();
  await expect(faultPanel).toContainText("暂无故障配置");

  await faultPanel.locator("select").nth(0).selectOption("sensor_bias");
  await faultPanel.locator("select").nth(1).selectOption("rl");
  await faultPanel.locator('input[type="number"]').fill("0.07");
  await faultPanel.getByRole("button", { name: "+ 添加" }).click();

  await expect(faultPanel).toContainText("RL · 传感器偏差 (0.070)");
  await expect(faultPanel.getByRole("button", { name: "停用" })).toBeVisible();

  await faultPanel.getByRole("button", { name: "停用" }).click();
  await expect(faultPanel.getByRole("button", { name: "启用" })).toBeVisible();

  await faultPanel.getByRole("button", { name: "启用" }).click();
  await expect(faultPanel.getByRole("button", { name: "停用" })).toBeVisible();

  await faultPanel.getByRole("button", { name: "清空全部故障" }).click();
  await expect(faultPanel).toContainText("暂无故障配置");

  await attachPageScreenshot(page, testInfo, "workflow-scenario-faults");
});

test("fault list failure does not masquerade as an empty fault configuration", async ({ page, request }, testInfo) => {
  await request.delete("/api/faults");
  await page.route("**/api/faults", async (route) => {
    if (route.request().method() !== "GET") {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic fault list failure" }),
    });
  });

  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await rail.getByRole("button", { name: /场景/ }).click();

  const faultPanel = page.locator(".panel").filter({ hasText: "故障注入" });
  await expect(faultPanel).toBeVisible();
  await expect(faultPanel).toContainText("读取故障失败：synthetic fault list failure");
  await expect(faultPanel.getByText("暂无故障配置")).toHaveCount(0);
  await expect(faultPanel.getByRole("button", { name: "+ 添加" })).toBeEnabled();
  await expect(faultPanel.getByRole("button", { name: "刷新故障" })).toBeEnabled();

  await attachPageScreenshot(page, testInfo, "workflow-fault-list-error");
});

test("script library workflow loads, starts, lays out markers, and stops a fixed script", async ({ page, request }, testInfo) => {
  await request.post("/api/script/stop");
  await request.post("/api/path/clear");

  await page.goto("/");
  await page.getByRole("tab", { name: "数据" }).click();

  const scriptPanel = page.locator(".panel").filter({ hasText: "动作脚本" });
  await expect(scriptPanel).toBeVisible();

  await scriptPanel.locator("select").selectOption("double_lane_change");
  await scriptPanel.getByRole("button", { name: "载入" }).click();
  await expect(scriptPanel.locator("textarea")).toContainText("name: double_lane_change");
  await expect(scriptPanel.locator("textarea")).toContainText("path_template: { name: double_lane_change");
  await expect(scriptPanel).toContainText("已载入: double_lane_change");

  await scriptPanel.getByRole("button", { name: "▶ 启动脚本" }).click();
  await expect(scriptPanel).toContainText("运行中: double_lane_change (7 个动作)");
  await expect(scriptPanel).toContainText("脚本名");
  await expect(scriptPanel).toContainText("double_lane_change");

  const pathResponse = await request.get("/api/path");
  expect(pathResponse.ok()).toBeTruthy();
  const pathBody = await pathResponse.json() as { name: string; points: unknown[]; cones: unknown[] };
  expect(pathBody.name).toBe("double_lane_change");
  expect(pathBody.points.length).toBeGreaterThan(5);
  expect(pathBody.cones.length).toBeGreaterThan(0);

  await scriptPanel.getByRole("button", { name: /停止脚本/ }).click();
  await expect(scriptPanel).toContainText("已停止");

  await attachPageScreenshot(page, testInfo, "workflow-script-library");
});

test("script parse failure stays visible in the script panel", async ({ page, request }, testInfo) => {
  await request.post("/api/script/stop");

  await page.goto("/");
  await page.getByRole("tab", { name: "数据" }).click();

  const scriptPanel = page.locator(".panel").filter({ hasText: "动作脚本" });
  await expect(scriptPanel).toBeVisible();

  await scriptPanel.locator("textarea").fill("script:\n  name: broken\n  actions: [");
  await scriptPanel.getByRole("button", { name: "▶ 启动脚本" }).click();

  await expect(scriptPanel).toContainText("script parse error");
  await expect(scriptPanel.getByRole("button", { name: "▶ 启动脚本" })).toBeEnabled();
  await expect(scriptPanel.locator("textarea")).toContainText("actions: [");

  await attachPageScreenshot(page, testInfo, "workflow-script-parse-error");
});

test("script library failure does not masquerade as an empty script library", async ({ page, request }, testInfo) => {
  await request.post("/api/script/stop");
  await page.route("**/api/script/library", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic script library failure" }),
    });
  });

  await page.goto("/");
  await page.getByRole("tab", { name: "数据" }).click();

  const scriptPanel = page.locator(".panel").filter({ hasText: "动作脚本" });
  await expect(scriptPanel).toBeVisible();
  await expect(scriptPanel).toContainText("读取脚本库失败：synthetic script library failure");
  await expect(scriptPanel.locator("select")).toBeDisabled();
  await expect(scriptPanel.getByRole("button", { name: "载入" })).toBeDisabled();
  await expect(scriptPanel.getByRole("button", { name: "刷新库" })).toBeEnabled();
  await expect(scriptPanel.locator("textarea")).toContainText("name: my_script");

  await attachPageScreenshot(page, testInfo, "workflow-script-library-error");
});

test("script status failure stays visible without blocking script editing", async ({ page, request }, testInfo) => {
  await request.post("/api/script/stop");
  await page.route("**/api/script/status", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic script status failure" }),
    });
  });

  await page.goto("/");
  await page.getByRole("tab", { name: "数据" }).click();

  const scriptPanel = page.locator(".panel").filter({ hasText: "动作脚本" });
  await expect(scriptPanel).toBeVisible();
  await expect(scriptPanel.getByRole("alert")).toContainText(
    "读取脚本状态失败：synthetic script status failure",
  );
  await expect(scriptPanel.getByRole("button", { name: "刷新库" })).toBeEnabled();
  await expect(scriptPanel.locator("textarea")).toBeEditable();

  await attachPageScreenshot(page, testInfo, "workflow-script-status-error");
});

test("recording panel records samples and exports csv", async ({ page, request }, testInfo) => {
  await request.post("/api/recording/stop");

  await page.goto("/");
  await page.getByRole("tab", { name: "数据" }).click();

  const recordingPanel = page.locator(".panel").filter({ hasText: "数据录制" });
  await expect(recordingPanel).toBeVisible();
  await expect(recordingPanel.getByTestId("recording-status")).toContainText("空闲");
  await expect(recordingPanel.getByTestId("recording-export")).toBeDisabled();

  await recordingPanel.getByRole("button", { name: /开始录制/ }).click();
  await expect(recordingPanel.getByTestId("recording-status")).toContainText("录制中");
  await expect.poll(async () => Number(await recordingPanel.getByTestId("recording-samples").textContent()))
    .toBeGreaterThan(0);

  await recordingPanel.getByRole("button", { name: /停止录制/ }).click();
  await expect(recordingPanel.getByTestId("recording-status")).toContainText("空闲");
  await expect(recordingPanel.getByTestId("recording-export")).toBeEnabled();

  const downloadPromise = page.waitForEvent("download");
  await recordingPanel.getByTestId("recording-export").click();
  const download = await downloadPromise;
  expect(download.suggestedFilename()).toMatch(/^sim4wis_\d+\.csv$/);

  await attachPageScreenshot(page, testInfo, "workflow-recording-export");
});

test("recording export failure stays visible in the recording panel", async ({ page, request }, testInfo) => {
  await request.post("/api/recording/stop");

  await page.route("**/api/recording/export.csv", async (route) => {
    await route.fulfill({
      status: 500,
      contentType: "application/json",
      body: JSON.stringify({ detail: "synthetic recording export failure" }),
    });
  });

  await page.goto("/");
  await page.getByRole("tab", { name: "数据" }).click();

  const recordingPanel = page.locator(".panel").filter({ hasText: "数据录制" });
  await expect(recordingPanel).toBeVisible();

  await recordingPanel.getByRole("button", { name: /开始录制/ }).click();
  await expect.poll(async () => Number(await recordingPanel.getByTestId("recording-samples").textContent()))
    .toBeGreaterThan(0);
  await recordingPanel.getByRole("button", { name: /停止录制/ }).click();
  await expect(recordingPanel.getByTestId("recording-export")).toBeEnabled();

  await recordingPanel.getByTestId("recording-export").click();
  await expect(recordingPanel).toContainText("导出失败：synthetic recording export failure");
  await expect(recordingPanel.getByTestId("recording-export")).toBeEnabled();
  await expect(recordingPanel.getByTestId("recording-status")).toContainText("空闲");

  await attachPageScreenshot(page, testInfo, "workflow-recording-export-error");
});

test("analysis replay controls scrub selected run data", async ({ page }) => {
  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });

  await rail.getByRole("button", { name: /试验/ }).click();
  await expect(page.getByText("运行矩阵")).toBeVisible();

  await page.getByRole("button", { name: /运行（1 runs）/ }).click();
  await expect(page.getByText(/完成 1 runs/)).toBeVisible({ timeout: 30_000 });

  await page.getByRole("button", { name: /去分析页对比这些 runs/ }).click();
  await expect(page.getByText("KPI 对比")).toBeVisible();

  await page.getByRole("button", { name: /回放/ }).click();
  await expect(page.getByText("回放（幽灵车叠放 · 车轮显示实际转角）")).toBeVisible();
  await expect(page.getByTestId("replay-canvas")).toBeVisible();

  const timeline = page.getByTestId("replay-timeline");
  await expect.poll(async () => Number(await timeline.getAttribute("max"))).toBeGreaterThan(1);
  await setRangeValue(timeline, "1");

  await expect(page.getByTestId("replay-time")).toContainText(/^1\.00 /);
  await expect(page.getByTestId("replay-close")).toBeVisible();
});

test("analysis replay metadata failure keeps replay usable with a visible fallback", async ({ page }, testInfo) => {
  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });

  await rail.getByRole("button", { name: /试验/ }).click();
  await expect(page.getByText("运行矩阵")).toBeVisible();

  await page.getByRole("button", { name: /运行（1 runs）/ }).click();
  await expect(page.getByText(/完成 1 runs/)).toBeVisible({ timeout: 30_000 });

  await page.getByRole("button", { name: /去分析页对比这些 runs/ }).click();
  await expect(page.getByText("KPI 对比")).toBeVisible();

  await page.route("**/api/runs/*", async (route) => {
    const url = new URL(route.request().url());
    if (route.request().method() === "GET" && /^\/api\/runs\/[^/]+$/.test(url.pathname)) {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic replay metadata failure" }),
      });
      return;
    }
    await route.fallback();
  });

  await page.getByRole("button", { name: /回放/ }).click();
  await expect(page.getByText("回放（幽灵车叠放 · 车轮显示实际转角）")).toBeVisible();
  await expect(page.getByText("回放尺寸读取失败，使用默认 LS9 尺寸：synthetic replay metadata failure")).toBeVisible();
  await expect(page.getByTestId("replay-canvas")).toBeVisible();

  const timeline = page.getByTestId("replay-timeline");
  await expect.poll(async () => Number(await timeline.getAttribute("max"))).toBeGreaterThan(1);
  await setRangeValue(timeline, "1");
  await expect(page.getByTestId("replay-time")).toContainText(/^1\.00 /);

  await attachPageScreenshot(page, testInfo, "workflow-replay-meta-error");
});

test("command palette filters and navigates workflow pages from the keyboard", async ({ page }) => {
  await page.goto("/");
  const rail = page.getByRole("navigation", { name: "工作流" });

  await page.keyboard.press("Control+K");
  const palette = page.getByRole("dialog", { name: "命令面板" });
  await expect(palette).toBeVisible();

  const search = page.getByLabel("命令搜索");
  await expect(search).toBeFocused();
  await search.fill("analysis");
  await expect(page.getByRole("button", { name: /去 分析/ })).toBeVisible();

  await page.keyboard.press("Enter");
  await expect(palette).toBeHidden();
  await expect(rail.getByRole("button", { name: /分析/ })).toHaveClass(/active/);
  await expect(page.getByText(/Run 库/)).toBeVisible();
});

test("command palette model switch failure surfaces a toast and closes cleanly", async ({ page }) => {
  await page.route("**/api/model", async (route) => {
    if (route.request().method() === "POST") {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ detail: "synthetic command palette model failure" }),
      });
      return;
    }
    await route.fallback();
  });

  await page.goto("/");
  await page.keyboard.press("Control+K");
  const palette = page.getByRole("dialog", { name: "命令面板" });
  await expect(palette).toBeVisible();

  const search = page.getByLabel("命令搜索");
  await search.fill("multibody");
  await expect(page.getByRole("button", { name: "切换模型：多体(14DOF)" })).toBeVisible();

  await page.keyboard.press("Enter");
  await expect(palette).toBeHidden();
  await expect(page.getByText("切换失败：synthetic command palette model failure").first()).toBeVisible();
  await expect(page.getByRole("navigation", { name: "工作流" })).toBeVisible();
});

test("gamepad mapping panel edits persist without a physical controller", async ({ page }) => {
  await page.goto("/");

  await expect(page.getByText("手柄映射与校准")).toBeVisible();
  await page.getByLabel("启用手柄输入").check();
  await expect(page.getByLabel("启用手柄输入")).toBeChecked();

  await page.getByTestId("gp-preset-per_wheel").click();
  await expect(page.getByTestId("gp-preset-per_wheel")).toHaveClass(/on/);
  await expect(page.getByTestId("gp-channel-fl")).toContainText("FL 左前");
  await expect(page.getByTestId("gp-channel-rr")).toContainText("RR 右后");
  await expect(page.getByText(/已接管策略.*manual_wheel/)).toBeVisible();

  await page.getByLabel("油门来源").selectOption("axis");
  await expect(page.getByRole("button", { name: /绑定（当前轴 1）/ })).toBeVisible();

  await setRangeValue(page.getByLabel("手柄死区"), "0.2");
  await expect(page.getByText("死区 0.20")).toBeVisible();

  await page.getByTestId("gp-reset").click();
  await expect(page.getByText("死区 0.08")).toBeVisible();
  await expect(page.getByTestId("gp-preset-per_wheel")).toHaveClass(/on/);
});
