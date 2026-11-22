# MATLAB 建模与代码生成

- `Tool` 提供腿部运动学、VMC 和补偿模型。运行 `build_leg_tool` 生成 `Wheel_Leg_Code/Code/Math` 所用的 C 文件，运行 `verify_leg_tool` 验证。
- `mixmpc_pre` 提供融合 MPC 模型、求解器、验证脚本和部署工具。运行 `build_mixmpc_pre` 完整构建，只有 Q/R 权重变化时可用 `build_mixmpc_pre_fast`。默认将六个生成文件部署到 `Wheel_Leg_Code/Code/Model`。

入口与构建说明见 [Tool](Tool/README.md) 和 [融合 MPC](mixmpc_pre/README.md)。
