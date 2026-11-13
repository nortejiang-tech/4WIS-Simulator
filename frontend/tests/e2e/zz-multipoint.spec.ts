import { test, expect } from '@playwright/test';

test('multipoint UI preserves parameters and executes simultaneous and staggered timelines', async ({ page, request }, info) => {
  const errors: string[] = [];
  page.on('pageerror', e => errors.push(e.message));
  page.on('console', m => { if (m.type() === 'error') errors.push(m.text()); });
  await request.delete('/api/faults');
  await request.post('/api/model', { data: { model_type: 'simplified_dynamic' } });
  await request.post('/api/interaction/control', { data: { action: 'pause' } });
  await page.goto('/');
  await expect(page).toHaveTitle(/4WIS/);
  await page.getByRole('navigation', {name: '工作流'}).getByRole('button', {name: /场景/}).click();
  const panel = page.locator('.panel').filter({hasText: '故障注入'});
  await expect(panel.getByText('草稿时间线（本地，未提交）')).toBeVisible();
  await panel.getByRole('button', {name: '双前轮同时',exact:true}).click();
  await panel.getByLabel('第 1 条起始延时秒').fill('0.023');
  await panel.getByLabel('第 2 条起始延时秒').fill('0.023');
  await panel.getByRole('button', {name:'应用时间线',exact:true}).click();
  await expect(panel.getByRole('button',{name:'停用'})).toHaveCount(2);
  let schedule = await (await request.get('/api/faults/schedule')).json();
  expect(schedule.faults[0].t_start).toBe(schedule.faults[1].t_start);
  // Start/pause simulation through the same product interaction API.
  await request.post('/api/interaction/control', { data: { action: 'resume' } });
  await expect.poll(async () => (await (await request.get('/api/faults/schedule')).json()).events.length).toBeGreaterThanOrEqual(2);
  await request.post('/api/interaction/control', { data: { action: 'pause' } });
  schedule = await (await request.get('/api/faults/schedule')).json();
  expect(schedule.events.filter((e: {event:string})=>e.event==='activated')[0].time).toBeCloseTo(schedule.faults[0].t_start, 9);
  await panel.getByRole('button', {name:'前后轮错时',exact:true}).click();
  await panel.getByRole('button', {name:'应用时间线',exact:true}).click();
  await expect.poll(async () => (await (await request.get('/api/faults/schedule')).json()).faults[1].wheel).toBe('rr');
  schedule = await (await request.get('/api/faults/schedule')).json();
  expect(schedule.faults[1].t_start-schedule.faults[0].t_start).toBeCloseTo(2, 10);
  await panel.screenshot({path:info.outputPath('multipoint-desktop.png')});
  await info.attach('multipoint', {body: await panel.screenshot(),contentType:'image/png'});
  // Builder must keep user-entered passive parameters, not replace them with defaults.
  await page.reload();
  await page.getByRole('navigation', {name: '工作流'}).getByRole('button', {name: /场景/}).click();
  await panel.getByLabel('草稿故障类型').selectOption('motor_free_caster');
  await panel.getByLabel(/自由转向参数 转向惯量/).fill('7');
  await panel.getByLabel(/自由转向参数 转向阻尼/).fill('91');
  await panel.getByRole('button', {name:'+ 加入时间线'}).click();
  await panel.getByRole('button', {name:'应用时间线',exact:true}).click();
  await expect.poll(async () => (await (await request.get('/api/faults/schedule')).json()).faults[0].j_steer).toBe(7);
  schedule = await (await request.get('/api/faults/schedule')).json();
  expect(schedule.faults[0].c_damp).toBe(91);
  expect(errors).toEqual([]);
  // Rejected overlap must keep previous schedule intact.
  await panel.getByRole('button', {name:'+ 加入时间线'}).click();
  await panel.getByRole('button', {name:'应用时间线',exact:true}).click();
  await expect(panel.getByRole('alert')).toContainText('故障时间窗口冲突');
  expect((await (await request.get('/api/faults/schedule')).json()).faults).toEqual(schedule.faults);
  await page.setViewportSize({width:768,height:1024});
  await expect(panel).toBeVisible();
  await panel.screenshot({path:info.outputPath('multipoint-compact.png')});
  // Chromium logs the deliberate HTTP 422 rejection above as a resource error.
  expect(errors).toHaveLength(1);
  expect(errors[0]).toContain('status of 422');
  await request.delete('/api/faults');
  await request.post('/api/interaction/control', { data: { action: 'resume' } });
});
