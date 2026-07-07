import { expect, test } from "@playwright/test";

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
