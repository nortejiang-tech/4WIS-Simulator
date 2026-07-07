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
  await expect(page.getByText(/还没有 run|从左侧选择/)).toBeVisible();

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
