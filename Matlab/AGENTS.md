# MATLAB 建模与构建维护约束

修改融合控制器前阅读 [模块约束](mixmpc_pre/AGENTS.md) 和 [构建维护说明](mixmpc_pre/docs/BUILD_FAILURES.md)。

- 保留用户已有修改。构建修复不应改变 Q/R、物理标定、控制周期或求解预算。
- `build/` 是可重建产物。完整入口应从无缓存的源码开始生成全部依赖。仅调 Q/R 时可运行 fast，保留缓存来源与完整性检查。
- 相对输出路径使用 MATLAB 调用时的 `pwd`，在切换目录前经 `mpc_output_path` 解析。两个入口均恢复调用者的 path、pwd 和输出环境变量。
- 完整与 fast 默认发布到固件 `Code/Model`。验证迁移时可指定独立输出目录。构建不会烧录固件。
