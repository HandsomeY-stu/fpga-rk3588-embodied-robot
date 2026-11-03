# 离线重放与资格审计

这些工具只读取录制输入并计算输出，不发送车辆命令。原始实车日志/图像不放入公开提交，使用者提供自己的录制目录。

在仓库根运行，`<run>`应包含同次录制的`control.jsonl`、`vision.jsonl`与`telemetry.jsonl`：

```bash
python3 software/task5_follow/audit_qualify.py --run <run> --out <audit-output>
python3 software/task5_follow/replay_strict.py --run <run> --compare-to-control --out <baseline-output.json>
python3 software/task5_follow/replay_strict.py --run <run> --mode qualified-fixed --cap 14 --q-tlm-age 280 --out <candidate-output.json>
python3 software/task5_follow/replay_compare.py <run> --profile BAG
```

必须先解释基线与原控制日志的逐拍差异，再引用候选收益。离线反事实冻结了视觉输入与车辆状态，不能据此宣称真实行驶改善。

其余argparse工具`replay_ab.py`、`replay_g0.py`和`replay_trim_cap.py`用于旧格式统计/诊断，各自`--help`说明输入。`analyze_stops.py <runs-parent>`读取该目录下的各次录制并就地输出CSV/JSON，使用录制副本以保留原始文件。`replay_trim_cap.py`含历史兼容默认值，不可替代严格重放的自证。

未恢复旧`gate_offline_test.py`：它硬编码已废弃的本机工程路径及旧门控结构。当前硬件控制边界使用`sim/rtl/tb_remote34.v`和`tb_task4_controls.v`验证，避免把旧源码文本检查当成当前RTL测试。
