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

test("experiment run can be handed to analysis", async ({ page }) => {
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

  await timeline.evaluate((el) => {
    const input = el as HTMLInputElement;
    const setter = Object.getOwnPropertyDescriptor(window.HTMLInputElement.prototype, "value")?.set;
    setter?.call(input, "1");
    input.dispatchEvent(new Event("input", { bubbles: true }));
    input.dispatchEvent(new Event("change", { bubbles: true }));
  });

  await expect(page.getByTestId("replay-time")).toContainText(/^1\.00 /);
  await expect(page.getByTestId("replay-close")).toBeVisible();
});
