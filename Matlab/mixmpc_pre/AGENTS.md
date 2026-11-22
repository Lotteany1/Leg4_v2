# 融合 MPC 构建维护

修改前阅读 [构建报错与维护约束](docs/BUILD_FAILURES.md)，并遵守上级 `Matlab/AGENTS.md` 中适用的路径、缓存和工作区保护约束。

- 日常入口是 `build_mixmpc_pre.m` 和 `build_mixmpc_pre_fast.m`。不要把它们与旧双 MPC 的两个模块混淆；先定位用户报错堆栈。
- 不为通过构建修改用户 Q/R、关闭 `ROLL_TIP_GAIN`、跳过 S8/G19 或放宽 20% 比较门限。
- 横滚倾覆项是模型外等效扰动。运行时预测、LQR 对照、S8 线性 plant、S10 非线性 plant 必须使用同一 `tinympc14_roll_force` 合同；不能只更新预测而遗漏 plant，或重复加到电机输出。
- 完整构建必须通过 S8、S10、调度 C 和求解器 C 对拍；fast 默认须通过横滚扰动合同、调度及求解器 C 对拍。改代码后先完整构建刷新本目录缓存，再跑 fast。
- 完整和 fast 使用调用时 MATLAB `pwd` 解析相对输出路径，保持 path/pwd 清理器及来源哈希检查。
- 保留顶层五个 MATLAB 入口；控制器继续叫 `tinympc14`，发布 solver/data 的两组 `.c/.h` 及 schedule/cost 两个 `.c`；完整和 fast 默认同步到固件 `Code/Model`（用户已明确授权）。URDF 仅作显式校核。

- 修改生成段写入时运行 `tinympc14_section_check`；保留 UTF-8 整文件替换与失败报错，不得把 C 文件段插入打包头文件的保护宏中。
