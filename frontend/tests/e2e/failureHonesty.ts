// H3 — the failure-honesty contract, as a helper instead of 25 copies.
//
// The e2e suite has a repeated shape: an API is made to fail, and the panel
// must (1) show the failure with the backend's own detail text, (2) NOT show
// the empty-state text that would dress the failure up as "nothing here",
// (3) keep its controls enabled, and (4) keep its existing content visible.
// Those four assertions *are* the contract; each panel that repeats them by
// hand risks drifting on one of the four. New panels should reach for these
// helpers so the contract is uniform by construction.
//
//   failApi(page, "**/api/experiments", "synthetic library failure");
//   await expectFailureHonesty(page, {
//     error: "读取实验库失败：synthetic library failure",
//     notMasqueradingAs: ["暂无已保存实验"],
//     staysEnabled: [library.getByRole("button", { name: "刷新" })],
//     staysVisible: [page.getByText("运行矩阵")],
//   });

import { expect, type Locator, type Page } from "@playwright/test";

/** Fulfil `pattern` with a 500 carrying `detail`, optionally for one method. */
export function failApi(
  page: Page,
  pattern: string,
  detail: string,
  opts: { method?: string; status?: number } = {},
) {
  return page.route(pattern, async (route) => {
    if (opts.method && route.request().method() !== opts.method) {
      await route.fallback();
      return;
    }
    await route.fulfill({
      status: opts.status ?? 500,
      contentType: "application/json",
      body: JSON.stringify({ detail }),
    });
  });
}

export interface FailureHonesty {
  /** The error line that must be visible — include the backend's detail
   * text, so the assertion proves the panel surfaces the reason, not just
   * an error badge. */
  error: string | RegExp;
  /** Empty-state texts that must NOT appear: showing one here is how a
   * failure masquerades as an empty library. */
  notMasqueradingAs?: (string | RegExp)[];
  /** Controls that must stay enabled — a failure must not brick the panel. */
  staysEnabled?: Locator[];
  /** Content that must stay visible — existing state survives the failure. */
  staysVisible?: Locator[];
}

/** Assert the four-part failure-honesty contract on one panel. */
export async function expectFailureHonesty(page: Page, honesty: FailureHonesty) {
  const error = page.getByText(honesty.error).first();
  await expect(error).toBeVisible();
  for (const text of honesty.notMasqueradingAs ?? []) {
    await expect(page.getByText(text)).toHaveCount(0);
  }
  for (const control of honesty.staysEnabled ?? []) {
    await expect(control).toBeEnabled();
  }
  for (const content of honesty.staysVisible ?? []) {
    await expect(content).toBeVisible();
  }
}
