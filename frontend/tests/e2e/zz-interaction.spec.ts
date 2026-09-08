import { expect, test, type Page } from '@playwright/test';
import fs from 'node:fs/promises';

const mode = (page: Page, name: string) => page.getByRole('navigation', { name: '交互方式' }).getByRole('button', { name, exact: true });

test.beforeEach(async ({ page, request }) => {
  await request.post('/api/interaction/control', { data: { action: 'stop' } });
  await request.post('/api/interaction/control', { data: { action: 'resume' } });
  await request.post('/api/recording/stop');
  await page.addInitScript(() => localStorage.setItem('4wis_quickstart_dismissed', '1'));
});

test('manual input releases on navigation, editing, blur and explicit stop', async ({ page }) => {
  let driver = { throttle: 0, steering: 0 };
  page.on('websocket', ws => ws.on('framereceived', ({ payload }) => {
    const data = JSON.parse(payload.toString());
    if (data.type === 'state') driver = data.driver;
  }));
  await page.goto('/');
  await expect(page.locator('.status.online')).toBeVisible();
  await page.keyboard.down('w');
  await expect.poll(() => driver.throttle).toBeGreaterThan(.2);
  await mode(page, 'AI Agent').click();
  await expect.poll(() => driver.throttle).toBe(0);
  await page.keyboard.up('w');
  await page.getByLabel('创建请求 · JSON').fill('wasd');
  await page.waitForTimeout(150);
  expect(driver.throttle).toBe(0);
  await mode(page, '手动驾驶').click();
  await page.keyboard.down('w');
  await expect.poll(() => driver.throttle).toBeGreaterThan(.2);
  await page.evaluate(() => window.dispatchEvent(new Event('blur')));
  await expect.poll(() => driver.throttle).toBe(0);
  await page.keyboard.up('w');
  await page.keyboard.down('w');
  await expect.poll(() => driver.throttle).toBeGreaterThan(.2);
  await page.getByRole('button', { name: '停止输入', exact: true }).click();
  await expect.poll(() => driver.throttle).toBe(0);
  await page.keyboard.up('w');
  await expect(page.getByRole('button', { name: '启用手动输入' })).toBeVisible();
});

test('script follows simulation clock, pauses with it and saves a full recording', async ({ page, request }) => {
  await page.goto('/');
  await mode(page, '脚本工况').click();
  await page.getByLabel('动作脚本 YAML').fill(`script:
  name: interaction_e2e
  actions:
    - { t: 0, action: drive, throttle: 0.1, steering: 0.02 }
    - { t: 10, action: stop }
`);
  const script = page.locator('.panel').filter({ has: page.getByLabel('动作脚本 YAML') });
  const recorder = page.locator('.panel').filter({ hasText: '数据录制' });
  await recorder.getByRole('button', { name: /开始录制/ }).click();
  await script.getByRole('button', { name: /启动脚本/ }).click();
  await expect.poll(async () => (await (await request.get('/api/script/status')).json()).running).toBe(true);
  await page.getByRole('button', { name: '暂停仿真', exact: true }).click();
  await expect(page.getByRole('button', { name: '继续仿真', exact: true })).toBeVisible();
  const before = await (await request.get('/api/script/status')).json();
  await page.waitForTimeout(250);
  const after = await (await request.get('/api/script/status')).json();
  expect(after.t_in_script).toBe(before.t_in_script);
  await page.getByRole('button', { name: '继续仿真', exact: true }).click();
  await expect.poll(async () => (await (await request.get('/api/script/status')).json()).t_in_script).toBeGreaterThan(before.t_in_script);
  await page.getByRole('button', { name: '停止输入', exact: true }).click();
  await expect.poll(async () => (await (await request.get('/api/script/status')).json()).running).toBe(false);
  await recorder.getByRole('button', { name: /停止录制/ }).click();
  const savedResponse = page.waitForResponse(r => r.url().endsWith('/api/recording/save'));
  await recorder.getByRole('button', { name: '保存到结果库', exact: true }).click();
  const saved = await (await savedResponse).json();
  expect(saved.complete).toBe(true);
  expect(saved.samples).toBeGreaterThan(0);
  const data = await (await request.get(saved.outputs.json)).json();
  expect(data.rows).toHaveLength(saved.samples);
  expect(data.rows[0]).toHaveProperty('tire_fx_fl');
  await expect(recorder.getByRole('button', { name: '完整 JSON', exact: true })).toBeVisible();
});

test('Agent uncertain response retries once and full artifacts reach analysis on desktop and narrow screens', async ({ page, request }, testInfo) => {
  const errors: string[] = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.goto('/');
  await mode(page, 'AI Agent').click();
  const created = page.waitForResponse(r => r.url().endsWith('/api/agent/sessions') && r.request().method() === 'POST');
  await page.getByRole('button', { name: '创建独立会话' }).click();
  const sid = (await (await created).json()).session_id;
  await expect(page.getByLabel('Agent 会话状态')).toContainText('0 样本');
  // The backend really accepts the first request; only its response is lost.
  const stepUrl = `**/api/agent/sessions/${sid}/step`;
  await page.route(stepUrl, async route => {
    const accepted = await request.post(route.request().url(), { data: route.request().postDataJSON() });
    expect(accepted.status()).toBe(200);
    await route.abort('failed');
  });
  await page.getByRole('button', { name: '推进 1.000 s', exact: true }).click();
  await expect(page.getByRole('button', { name: '重试同一请求' })).toBeEnabled();
  await page.unroute(stepUrl);
  await page.getByRole('button', { name: '重试同一请求' }).click();
  await expect(page.getByLabel('Agent 会话状态')).toContainText('200 样本');
  expect((await (await request.get(`/api/agent/sessions/${sid}`)).json()).steps).toBe(200);
  await page.getByLabel('转向 [-1,1]', { exact: true }).fill('0.05');
  await page.getByRole('button', { name: '推进 1.000 s', exact: true }).click();
  await expect(page.getByLabel('Agent 会话状态')).toContainText('400 样本');
  await page.getByRole('button', { name: '保存完整结果', exact: true }).click();
  const download = page.waitForEvent('download');
  await page.getByRole('button', { name: '完整 JSON', exact: true }).click();
  const file = await download;
  const full = JSON.parse(await fs.readFile((await file.path())!, 'utf8'));
  expect(full.rows).toHaveLength(400);
  expect(full.manifest.full_data_decimated).toBe(false);
  expect(full.rows.at(-1).t).toBeCloseTo(2, 10);
  await page.screenshot({ path: testInfo.outputPath('agent-desktop.png') });
  await page.setViewportSize({ width: 390, height: 844 });
  // Compare real element rectangles: body.scrollWidth hides clipped grid tracks.
  for (const selector of ['.app-header', '.interaction-bar', '.app-body', '.agent-config', '.agent-main', '.agent-command']) {
    const rect = await page.locator(selector).boundingBox();
    expect(rect).not.toBeNull();
    expect(rect!.x + rect!.width, selector).toBeLessThanOrEqual(390.5);
    expect(rect!.x, selector).toBeGreaterThanOrEqual(0);
  }
  await page.screenshot({ path: testInfo.outputPath('agent-mobile.png') });
  await page.setViewportSize({ width: 1280, height: 720 });
  await page.getByRole('button', { name: '在分析页打开', exact: true }).click();
  await expect(page.getByTestId('analysis-workbench')).toBeVisible();
  expect(errors).toEqual([]);
  await request.delete(`/api/agent/sessions/${sid}`);
});
