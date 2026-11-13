# v0.107.0 全 C++ 后端候选：最终软件验收

日期：2026-11-14。分支 `codex/cpp-full-backend`。代码和打包源提交 `e79ca99c4eb6fb9b78f5c4949506f4cd09436f18`；后续分支提交仅归档文档、证据、测试兼容修订和解压浏览器验证工具，不改变此发布二进制。物理数值核心与 A 分支相同。当前版本的 UI 仍为 React，内置仿真、控制、研究分析、HTTP/WebSocket/MCP 为独立 C++17；用户 Python/FMU/MATLAB 插件使用明确的独立兼容进程。

## 实际结果

|验收项|Mac arm64|Windows x64|证据|
|---|---|---|---|
|C++ 差分、领域、接口与 MCP 测试|238 PASS，26.06 s|238 PASS，0 fail / 0 skip，28.003 s|macos-tests.txt、windows-tests.xml|
|前端类型检查与单元测试|PASS；54 PASS|PASS；54 PASS|macos-frontend.txt、Windows CI|
|58 项浏览器流程|最终解压包 58 PASS，41.8 s|原生 C++ 服务 58 PASS|macos-packaged-browser.txt、Windows CI|
|解压便携包|10/10 PASS|10/10 PASS|package-macos.json、package-windows.json|
|六个冻结黄金工况|6 PASS|6 PASS|包检查中故意指定不存在的 Python，内置流程仍成功|
|三路线对照性能|8 组 × 5 次，全部通道对照 PASS|本版本未单独测三路线性能|benchmark-macos.json|
|用户目标机、GPU/输入硬件、长时间运行|PENDING|PENDING|没有以 CI 替代用户机器验收|

四个 Python 参考测试文件的 UTF-8 / CRLF 与测试桩兼容修订另经定向验证：93 PASS，23.10 s（reference-test-compatibility.txt）；没有更改发布应用。

Windows 最终工作流：[43491350201](https://github.com/nortejiang-tech/4WIS-Simulator/actions/runs/43491350201)，全部步骤成功。官方构建产物 SHA-256 `81e30e9ab086385fabd07b94a765170e622474919fcefb94f78371f606446c44` 下载后验证一致，ZIP CRC 通过。具体元数据见 windows-artifact.json。解压包检查覆盖中文与空格路径、生产前端、两模型/两轮胎多故障、会话重试幂等性、CSV/JSON/ZIP、官方 MCP SDK 初始化/18 工具发现/实际调用和包内可选插件解释器。

Mac 最终构建最低部署目标为 13.3，SDK 27.2；实际测试系统为 macOS 27.2.1，Apple M5 Pro / 64 GiB。13.3 是编译部署下限，较旧 macOS 的实际运行验收仍为 PENDING。此限制来自浮点 std::to_chars 可用版本，没有为了下探系统而更改数值格式化。版本与构建参数见 macos-build.json。

## 安装包身份

|包|SHA-256|
|---|---|
|4WIS_CPP_v0.107.0_macos-arm64.zip|bee22984762a89675ab9b4cd8614bff620a3246bae5136eab086053433d94411|
|4WIS_CPP_v0.107.0_windows-x64.zip|f1442a3b26d33865b2134c62e08d5dbca19145d2765ccee2736e2f981aa4d935|

Mac 二进制 SHA-256 `a7997d667a0a196c7e3d28fb94aef2930e92efe3b1217ebe89c1422b857c9a07`；Windows 二进制 `06b3cacc14e1e8562aad030c7228bcab8e052055c57f678d49612dbd2a7f4f6b`。二者引擎源指纹为 `bc560957e039b596c4d15ee75974724d48c1eba404e13825cc73d32788c74bc2`。性能报告测量的就是此 Mac 发布二进制。发布页的最终证据附件优先于压缩包内部的构建时文档快照。

## 失败处理与复核

初轮 Windows 中选型时长上限受 Python 3.11 / 3.12 求和舍入影响差一个积分步，导致 RMS 差 0.001 Nm。两路线采用相同的闭式行程预算与 ceil 步数，严格的选型判定容差未放宽。另两轮出现 HTTP 复用已关闭连接的 RemoteProtocolError；修复为响应明确声明 Connection: close，与服务实际连接策略相符。最终 238 项全部通过，没有增加请求重试掩盖故障。中文路径 CRUD、批作业历史上限及完成线程释放均有实际测试。

物理复核发现并修复选型载荷曲线混入其他车轮以及执行器转角/前束坐标混淆，见 sizing-correction.md 与 physics-review.md。正常轮荷、力/矩平衡、轮速角冲量、摩擦约束、机构虚功与多故障事件边界均有独立断言；数值差分和黄金曲线只能补充这些证据，不能独立证明实车正确性。

## 验收范围与回滚

软件验收在上述平台、接口及用例范围内 PASS。台架/实车关联、实测轮胎/机构参数、非对称双侧精确齿条负载、用户目标机长期 GUI、真实 FMU/MATLAB 模型/引擎/许可为 PENDING。传感器故障只改变观测；总体耦合仍为分区积分，不能宣称整体四阶。兼容进程提供故障/超时隔离，不构成不可信代码沙箱。

本版本作为独立 prerelease，main 和稳定 v0.105.0 不替换。停止当前服务并启动独立稳定目录即可切回；没有用户数据格式迁移，本轮未执行用户数据回滚。MCP 与本地浏览器默认端口为 8020；A 路线默认 8010，使用各自解压目录对比。EVO-Coder 为 0 调用、0 生成 tokens、0 可核实云端节省；主模型 tokens 不可获得，未估造。
