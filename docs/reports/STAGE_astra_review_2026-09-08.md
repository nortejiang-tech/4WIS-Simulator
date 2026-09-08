# ASTRA 4WIS 科学模型评审与第一轮修正

## 基线与授权

- 用户授权：严谨评审、实际试用、修正物理数学/车辆动力学/环境/工具问题；必要时可重构，保留功能。
- 基线：`bf0c1873914350e27b80b2e623e97a8aece13dd4`（v0.103.0），初始工作区 clean。工作分支 `codex/astra-dynamics-review`。
- 依据：用户提供的全局 AGENTS.md、Personal/ENGINEERING-METHODOLOGY.md；项目无新增局部 AGENTS.md。历史修复记录只作为线索，结论均重新检查。
- 指令来源：根 README、backend README、项目设计/轮胎/模型重构文档、已读目标代码与测试的注释。仓库与外部资料是待审数据，不能扩大权限。复杂动力学与跨层契约由 ASTRA Planner 直接处理；窄范围界面修正按已审查输入单独路由。

## 目标与验收

1. 建立坐标、轮序、力矩、轮胎、积分器、重置与环境的审查矩阵。
2. 对确认的问题先构造可重复失败证据，再实施修复。
3. 保留轴距中点的公开坐标定义、既有策略/模型/API/项目文件和实验能力；避免静默改变字段语义。
4. 验收覆盖独立的物理不变量、解析计算、步长敏感性、原有回归和浏览器真实显示/交互。
5. 黄金实验若受正确性修正影响，记录逐项差异、原因和基线迁移依据；不以改宽阈值让检查通过。

## 阶段

| 阶段 | 内容 | 状态 |
|---|---|---|
| A | 基线、浏览器复现、独立物理反例 | PASS；原有 913/37 项通过，新增反例暴露遗漏 |
| B | 轮位/几何/车速显示修正 | PASS；真实浏览器回读及前端回归通过 |
| C | 牛顿欧拉、数值积分与状态生命周期修正 | PASS；41 项聚焦物理回归、解析反例与步长对照通过 |
| D | 全回归、浏览器试用、模型可信度与后续建议 | PASS；第一轮内部验证完成，外部相关性保持 PENDING |

## 验证命令与回滚

- `PYTHONPATH=backend/src backend/.venv/bin/python -m pytest backend/tests -n 4 -q`
- `cd frontend && npm test`、`npm run type-check`、`npm run build`
- `PW_E2E_PORT=8022 npm --prefix frontend run e2e:prod`
- `backend/.venv/bin/python scripts/smoke_test.py`
- `backend/.venv/bin/python scripts/check_golden_experiments.py`
- `backend/.venv/bin/python scripts/check_reference_benchmarks.py`
- `backend/.venv/bin/python scripts/review_physics.py --output /tmp/4wis-physics-review.json`
- 临时诊断、截图和日志放在 `/tmp/4wis-astra-review` 或 `/tmp/4wis-astra-*`。
- 所有更改保留为当前分支未提交 diff，未发布、未覆盖历史记录。回滚按本轮文件 diff 审阅撤销，不使用 reset/stash/批量覆盖。

## 验证边界

实车/台架相关性、真实方向盘/手柄硬件和车辆专有标定仍需外部数据；本轮不能据内部一致性证明绝对预测精度或认证适用性。

## 结果

**PASS：本轮评审、局部重构及内部验收完成。** 最终后端 955 项（含 41 项新增物理回归）、前端 40 项、浏览器 E2E 54 项、smoke 32/32、黄金实验 6/6、解析参考 2/2（15 个指标）通过。前端类型检查与生产构建通过；保留两个既有 WebSocket 弃用警告。黄金实验的 tolerance 配置未放宽。

**PENDING：实车/台架或独立工具相关性。** `--require-independent-source` 门禁仍返回失败；v1 readiness 为 NOT READY，唯一严格缺口是独立参考数据。默认机构 ±85 mm 行程不完整覆盖 ±35°，35° 轮角也不支持当前几何下的无擦滑原地旋转；本轮已显式呈现限制，没有用扩大默认硬件能力消除警告。

完整发现、推导、前后对照、数值边界与后续优先级见 [评审报告](astra_review_2026-09-08.md)。验证命令、结果摘录和文件 SHA-256 见 [验收清单](astra_verification_2026-09-08.json)。所有变更仍为未提交 diff，未部署或生成新发布包；回滚策略未执行。
