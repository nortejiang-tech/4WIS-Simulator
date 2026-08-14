# targets/ — 你自己的需求集

把 `*.yaml` 放进这个目录，`sim4wis targets list` 就会看到它。
内置的两套（`eps_actuator`、`steering_feel`）在代码里，随程序一起发布，
这个目录是**在它们之上追加**，不是替换。

```bash
sim4wis targets list
sim4wis targets show <name>
sim4wis targets check <name>          # 对着一次作动器选型走查判定
sim4wis targets check <name> --margins \
           [--fit-record fit.json]    # 附参数空间余量：参数要错多少判定才翻（C5/C3c）
```

格式与写法见 [`docs/targets_guide.md`](../docs/targets_guide.md)，
可直接复制的示例见 [`docs/examples/targets_vehicle_response.yaml`](../docs/examples/targets_vehicle_response.yaml)。

三件容易踩的：

- **`source` 必填。** 需求评审第一个问题就是"谁说的"。
- **版本不能重用。** 文件里声明一个已内置的 `name@version` 会被直接拒绝 ——
  一个被引用的需求版本不能指向两份不同的文档。改了内容就 bump `version`。
- **`target` 必须落在 `limit` 里面。** 想达到的比不能越过的还松，是写错了。

也可以用 `$SIM4WIS_TARGETS_DIR` 指到别处（比如一个单独的需求仓库）。
