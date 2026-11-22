# s59_trial01 低腿倒车停稳搜索

以当前生产参数为中腿对照，以 s59_trial01 为搜索中心。试验使用隔离源码和上一份有效的轮矩时标录波给出的阻尼范围。低腿训练工况复现 0.19 m、−0.71 m/s 倒车、12 ms 执行延迟，松杆后以 0.10 m/s 和腿角门槛模拟驻车锁点，再爬升位置反馈权重。

先对八组纵向权重做单轴和拉丁超立方采样，再以首轮排名最靠前的候选为中心缩小范围。随后分别搜索腿角速度、腿角和速度误差权重与轮髋力矩代价。各次筛选直接运行约束 MPC 和非线性被控对象，逐工作点精确求解终端 Riccati 方程。独立工况改变腿长、车速、负载、延迟、俯仰扰动和横滚冲击，并包含中腿起停与驻车。

验收时检查有限状态、每个工况的姿态峰值、限幅与回退。姿态只给 0.002 rad 仿真比较公差。指定倒车工况须进入驻车，最后 0.55 s 内速度绝对值须保持在 0.06 m/s 以下。中腿正反向速度 RMS 逐工况与生产基线对照，起步和停稳时间给一个 2 ms 节点的比较公差。通过全部闭环门槛后才进入 fast、完整构建、C 对拍与 ARM 固件构建。当前实机数据用于定仿真范围，实机效果仍由后续 A/B 验证。

## 结果

共筛选 88 组权重，15 组入围候选各复核 17 个工况。所有复核均有限，但没有候选同时通过停稳、姿态和中腿跟踪门槛。已核对生产配置和固件源码哈希，均与试验前一致。本轮未触发候选固件构建。

| 候选 | 标称 19 cm 松杆到停稳 | 标称末段速度峰值 | 主要验证结果 |
|---|---:|---:|---|
| 生产基线 | 观察窗内未停稳 | 0.286 m/s | 中腿和姿态对照 |
| s59_trial01 | 观察窗内未停稳 | 0.115 m/s | 搜索中心 |
| axis08_1 | 观察窗内未停稳 | 0.101 m/s | 守住中腿和姿态门槛，五个重点倒车工况仍有四个未完全停稳 |
| brake01 | 0.534 s | 0.0098 m/s | 中腿速度跟踪改善，扰动倒车工况未全部停稳，姿态超出门槛 |
| lean11 | 0.732 s | 0.0088 m/s | 守住姿态门槛，仍有一个扰动倒车工况未停稳，中腿松杆停稳变慢 |
| track11 | 0.770 s | 0.0148 m/s | 标称姿态通过，25 cm 停稳由 0.556 s 延至 0.672 s，独立复验仍有姿态和停稳问题 |

[常规搜索](build/low_stop_refine/results.json) · [制动搜索](build/low_stop_refine/braking_results.json) · [姿态搜索](build/low_stop_refine/attitude_results.json) · [跟踪搜索](build/low_stop_refine/tracking_results.json)

## 复跑

从仓库根目录依次执行。各轮搜索共用隔离源码，并各自记录候选和结果。

    python -B Matlab/mixmpc_pre/pso/refine_low_stop.py --workers 3
    python -B Matlab/mixmpc_pre/pso/refine_low_stop.py --directed --workers 3
    python -B Matlab/mixmpc_pre/pso/refine_low_stop.py --attitude --workers 3
    python -B Matlab/mixmpc_pre/pso/refine_low_stop.py --tracking --workers 3

输出位于 build/low_stop_refine。使用相同的方案和目录可续跑已完成的场景。同机分批运行 MATLAB 时，先让一支分支结束再运行另一支。
