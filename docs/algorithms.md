# 关键算法与代码阅读顺序

## 1. 流水图像处理

[五点中值](../fpga/src/despeckle/despeckle_median5.v)按水平窗口进行比较，网络拆为两级以缩短组合路径。边缘像素、窗口推进与valid延迟需作为一个整体理解，不能单独改数据通路延迟。

[RGB补偿](../fpga/src/rgb_neutral_trim.v)用LUT4实现RGB565到RGB888的舍入、增益和饱和映射：红通道45/32、蓝通道34/32，绿通道展开。实现保持组合接口，不增加BRAM/DSP。传感器自动白平衡/曝光/增益仍启用，固定补偿不能替代光照标定。

## 2. 运动闭环与差速

[运动控制](../fpga/src/motion/motion_ctrl.v)将百分比速度转为轮目标速度，结合编码器测量和[定点PID](../fpga/src/motion/pid_ctrl.v)形成闭环。

行进纠偏采用一侧轮半速：理想情况下`v_outer=V`、`v_inner=V/2`，车身均速约`0.75V`。提高该档速度会同时提高角速度，因此策略分别限制直线速度、纠偏速度与不可靠目标续行速度。

## 3. 任务四角度与尾段

[任务四状态机](../fpga/src/task4/task4_seg1_fsm.v)对`target-current`进行±180°规约，避免跨边界时直接比较绝对航向。v48追加尾段采用绝对误差判停，随后进行有界微修正；D6/D7/D8为600/1000/500mm。

默认微修正计算等价于：

```text
n = clamp(floor(abs_error * 10 / 1024), 1, 3)
abs_error < 205 → 1
abs_error < 308 → 2
其余 → 3
```

阈值实现避免乘法DSP；全有符号18位输入已经与默认原算法穷举比较。非默认最大修正拍数保留兼容分支。

## 4. 目标关联和双阈值

任务五[RedBottleLock](../software/task5_follow/red_bottle_target.py)先按语义、颜色和框合法性筛候选，再结合IoU、尺度和空间连续性匹配。首次获取使用较高门槛；已建立轨迹的低分续跟仍需满足同一物体的连续性约束。

重复检测框先折叠；多匹配时结合已有目标选择。RED_HOLD只维护已建立的身份，带空间、尺度和时间边界。低分续跟和红色维持不等同于满足高速资格。

## 5. 统一决策出口

[FollowPolicy](../software/task5_follow/follow_policy.py)阅读顺序：`step` → 目标状态 → `_req_track/_req_lost` → `_shape` → `_guard`。请求速度、整形速度和最终发令分别记录，便于区分控制器主动降速与外部保护。

`_update_qualify`按不同新鲜帧累积真实目标可靠性，结合置信度、距离和输入时效判定。`_guard`在发令前统一复核，避免某个目标分支提前返回而绕过保护。

## 6. 红瓶搜索靠近

任务六[策略](../software/task6_red_bottle/red_bottle_target.py)综合红色比例和bottle类别确认目标。检测器[红色统计](../software/detector/src/realtime_ssd.cpp)提供全帧及左右半幅的方位线索。策略保留书包侧记忆，减少语义标签暂时消失后再次误跟同侧红色书包。

目标可见时边行进边差速对准；到达距离/画面尺度门槛后停止。小目标低分、遮挡、背景红色和目标重入仍是应单独验证的场景。
