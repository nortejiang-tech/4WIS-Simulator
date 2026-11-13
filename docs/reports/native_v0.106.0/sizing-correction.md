# 选型载荷曲线修正（两个候选分支共同修复）

迁移交叉核对发现：原 `_rack_force_curve` 将 `sweep_load_analysis` 返回的四个车轮行全部混入同一插值表。未转向车轮的重复零转角对应不同受力，导致排序顺序影响选型结果。这是原实现问题，不能通过放宽 C++ 容差接受。

修复只取指定 FL 车轮，并以 `delta_cmd`（含静态前束前的执行器坐标）作为插值横轴。原选型模型继续采用单侧负载镜像到两根拉杆的简化假设；它不是左右独立 K&C、非对称载荷、真实齿条刚度或实车标定的替代。停驶擦地阻力仍进入库仑阻力项，滚动回正负载仍按方向进入负载项。

新增 3 个不同车速的受力来源回归：注入极大的其他车轮负载也不能污染 FL 曲线；静态前束也不能平移执行器坐标。现有选型、轮端选型、目标判定共 82 项通过（2026-11-14）。既有大小电机比较原用 3 N·m/8 N·m，修正后两者都不饱和，因此手力矩本应相同。改用实测有 22.499% 助力饱和、手力矩未限幅的 2 N·m 电机作对照，并增加饱和状态断言；其峰值手力矩 9.2783 N·m，8 N·m 电机为 2.8693 N·m。

该修正不改变车辆数值内核、故障调度或性能测试工况，原 Windows/Mac 数值指纹仍适用。最终安装包必须包含修正后的 Python 选型源文件。

Windows 第三轮：后端 1119、前端单测 54、浏览器 58、8 组五次重复性能测量通过；原生打包因 METADATA 的 CRLF 换行被错误识别为版本不符而失败，已改为按行解析。最终包验证另行记录，不能将该轮整体写为 PASS。

## Cross-version duration bound

Windows Python 3.12 and Mac Python 3.11 exposed a one-step discrepancy in the
sizing run time cap: `sum()` changed its floating point summation algorithm,
and truncation at an integer boundary altered an RMS result by 0.001 N·m.
Both routes now use the closed-form sum of the conservative leg-distance bound
and round the upper time budget up. This keeps the waypoint trajectory and
inner integration unchanged while removing version-dependent accumulated sums.
C++ uses the same radians multiplication order as Python for these limits.
The strict sizing/compliance comparison is retained, not widened.
