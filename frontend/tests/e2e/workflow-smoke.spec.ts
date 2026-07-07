import { expect, test, type Locator } from "@playwright/test";

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

test("workflow rail pages render from the production app shell", async ({ page }) => {
  await page.goto("/");

  await expect(page.locator(".brand")).toHaveText("4WIS");
  await expect(page.locator(".title")).toHaveText("Simulator");
  const rail = page.getByRole("navigation", { name: "工作流" });
  await expect(rail).toBeVisible();
  await expect(rail.getByRole("button", { name: /运行/ })).toHaveClass(/active/);

  await rail.getByRole("button", { name: /试验/ }).click();
  await expect(page.getByText("运行矩阵")).toBeVisible();

  await rail.getByRole("button", { name: /分析/ }).click();
  await expect(page.getByText("还没有 run — 到「试验」页跑一个批量")).toBeVisible();
  await expect(page.getByText(/从左侧选择 1.*6 个 run/)).toBeVisible();

  await rail.getByRole("button", { name: /车辆/ }).click();
  await expect(page.getByText("车辆 / 悬架参数").first()).toBeVisible();
  await expect(page.getByRole("img", { name: "整车俯视参数示意" })).toBeVisible();

  await rail.getByRole("button", { name: /场景/ }).click();
  await expect(page.getByText("场景").first()).toBeVisible();
  await expect(page.getByText("轨迹 / 路径").first()).toBeVisible();

  await rail.getByRole("button", { name: /负载/ }).click();
  await expect(page.getByText("实时四轮负载")).toBeVisible();

  await rail.getByRole("button", { name: /原理/ }).click();
  await expect(page.getByText("4WIS foundation model")).toBeVisible();
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
  await expectCanvasHasDrawnPixels(page.getByTestId("analysis-chart-rack_force_fl").locator("canvas").first());

  const downloadPromise = page.waitForEvent("download");
  await page.getByTestId("analysis-chart-png-rack_force_fl").click();
  const download = await downloadPromise;
  expect(download.suggestedFilename()).toBe("rack_force_fl.png");

  const screenshot = await page.getByTestId("analysis-workbench").screenshot();
  expect(screenshot.length).toBeGreaterThan(10_000);
  await testInfo.attach("analysis-workbench", { body: screenshot, contentType: "image/png" });
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
