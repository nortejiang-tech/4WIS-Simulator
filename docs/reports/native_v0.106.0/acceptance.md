# v0.106.0 C++ 仿真内核候选：最终软件验收

日期：2026-11-14。分支 `codex/native-simulation-core`，基线稳定 v0.105.0。最终应用/数值测试源提交 `85202cde071aea1fcfc4e0b41ec9a638898afd8b`；Windows 打包修正提交 `e157b8927c8f3617e91cac20315ded9fa67cb572` 只补齐运行库和验证流程，没有改变数值源或二进制。后续分支提交归档最终文档、证据与复现工具。

## 实际结果

|验收项|Mac arm64|Windows x64|证据|
|---|---|---|---|
|最终完整后端回归|1122 PASS，164.89 s|1122 PASS，0 fail / 0 skip|macos-final-backend.txt、windows-tests.xml / windows-validation.txt|
|前端类型检查与单元测试|PASS；54 PASS|PASS；54 PASS|local-validation.txt、Windows CI|
|浏览器 E2E|最终解压包 58 PASS，47.6 s|原生服务 58 PASS，2.3 min|macos-packaged-browser.txt、windows-validation.txt|
|最终便携包|43/43 PASS|49/49 PASS|macos-package.json、windows-package.json|
|五次交替运行的性能与逐通道对照|8 组 PASS|8 组 PASS|benchmark-macos.json、benchmark-windows.json|
|冻结黄金与解析基准|6 个黄金；2 组解析/15 指标 PASS|完整回归与解析基准 PASS|local-validation.txt、Windows CI|
|新打包回归|3 PASS|3 PASS|运行库副本与来源、不存在/歧义来源拒绝|
|用户目标机长期 GUI、硬件输入和车辆验收|PENDING|PENDING|未以软件回归替代实际设备验收|

最终完整 Windows 工作流：[43488657384](https://github.com/nortejiang-tech/4WIS-Simulator/actions/runs/43488657384)，全部步骤成功。完整测试产物官方 SHA-256 为 `4d2d09c6ba71e750c9bb5dfca7cfd20880b4baaa33a8e09b5d71bda957582053`，下载后核对一致。

发布前 PE 导入检查发现扩展依赖 MSVCP140.dll，而原包只包含 NumPy 的哈希后缀 DLL，无法按扩展的导入名称满足依赖。开发机的系统预装可能掩盖遗漏。现在将包内已有 NumPy 运行库保留原文件和许可证，再按导入名称复制到 Python 同目录并记录 SHA-256。遵循 [Microsoft 本地部署说明](https://learn.microsoft.com/en-us/cpp/windows/deployment-in-visual-cpp?view=msvc-170)。专项工作流 [43496625894](https://github.com/nortejiang-tech/4WIS-Simulator/actions/runs/43496625894) 复用上轮已验证的同一个原生 wheel，重新打包；普通导入扩展后用 GetModuleFileNameW 断言实际加载路径来自解压包，49 项全部通过。不是仅检查 DLL 文件存在，也没有更改系统运行库。最终产物官方 SHA-256 `d4053cfeda338eaf844151251ce4eecae7c007197d4534cb0150a59bc9bb849d` 与下载文件一致。两轮身份见 windows-artifact.json。

Mac 实际运行平台为 Apple M5 Pro / 64 GiB、macOS 27.2.1；最终包内使用 CPython 3.12。日常开发参考 Python 3.11 与包内 Python 3.12、C++ 选型时间预算的交叉结果见 sizing-cross-version.txt。数值核心源指纹为 `59c3cb2b411f4c5d1471fe43695b8cd281e663a1e96069788058ef732a712fc4`，全部发布原生扩展和性能测试保持一致。

## 安装包身份

|包|SHA-256|
|---|---|
|4WIS_Simulator_v0.106.0_macos-arm64.zip|db23ce37c87388d27cc79e6325693a331807be85a70493a0db94b9a4382a5386|
|4WIS_Simulator_v0.106.0_windows-x64.zip|13ec4052ba14fe42d2e793ff6edc7a6581a1593352cee2c06806df73f93dcfc6|

发布页最终证据附件优先于 ZIP 内部的构建时文档快照。样例的同时/错时多点故障定义、各次耗时与全部通道差值保留在 benchmark JSON；跨路线样例工程解释见 B 分支的 comparison.html。

## 失败历史与未关闭的边界

前期 Windows 中 UTF-8 文件读写、CRLF wheel 元数据识别已修复；不同系统的浏览器截图有各自基线。动画周期检查使用共同 960×540 视口，仍保留 >150 帧、6–34 ms 中位帧间隔、最坏自相关 <0.35 的原判据。第四轮 [43485693370](https://github.com/nortejiang-tech/4WIS-Simulator/actions/runs/43485693370) 有一次最坏自相关 0.5585516 的 FAIL（57/58）；最终完整工作流相同前端为 58/58。该次帧时序异常原因未独立定位，没有把再次通过写成已彻底修复；目标机 GPU/长时间流畅性仍为 PENDING。失败摘录保留在 windows-frame-timing-history.txt。

选型载荷曲线混入其他轮受力，以及 Python 版本求和造成时间上限少一步的问题，均在 A/B 两路线共同修复，严格选型容差未放宽。物理证据见 physics-review.md、sizing-correction.md 与相关独立力/矩、角冲量、摩擦和虚功测试。核心迁移逐通道一致并不等于真实车辆已验收。

软件验收仅在上述用例、平台与一次运行范围内 PASS。台架/实车关联、真实轮胎/机构参数、目标机实际输入设备及长期使用为 PENDING。总体模型为分区积分，传感器故障只改变观测，选型保留单侧负载对称镜像近似；未操作物理 CAN 或车辆。

## 发布和回滚

独立 prerelease，不合并 main，不替换稳定 v0.105.0。默认端口 8010；另一条完整 C++ 后端路线默认 8020，使用各自目录。可用 SIM4WIS_ENGINE=python 切回保留的参考实现，或停止当前候选后启动独立稳定包；没有用户数据格式迁移，本轮没有用户数据回滚操作。用户原有 draft_03c081a7_folder 未动、未提交。

EVO-Coder 本次 0 调用、0 生成 tokens、0 可核实云端节省；主模型实际 tokens unavailable，未估造。Markdown/JSON 账本见 docs/reports/evo-coder。
