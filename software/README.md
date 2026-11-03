# RK3588软件

## 模块

| 目录 | 内容 | 入口 |
|---|---|---|
| detector | 常驻RKNN推理、YOLO解码、红色区域统计、V4L2和UART | `make`生成`realtime_yolo` |
| task5_follow | 指定目标跟随、可靠性分级、连续行进纠偏 | `run_follow_trial.py` |
| task6_red_bottle | 红瓶搜索靠近、书包干扰排除、网页/TCP服务 | `run_red_bottle.py`、`webctl.py` |
| runtime | 网页JPEG采集和链路控制 | `grab_loop2.sh`、`vision_ctl.sh` |

两个任务目录中的`red_bottle_target.py`、`prop_ctrl.py`和`webctl.py`属于不同功能快照。通过各自目录启动，避免Python导入到另一版本。`run_b.py`、`run_c.py`等入口保留用于兼容既有导航调用，首次使用优先选择上表入口。

Python控制代码使用标准库；无需为了运行控制策略安装桌面图形依赖。C++检测器需要Linux V4L2、C++11编译器和匹配的RKNN运行库。模型输入采用640×640，输出再映射至采集坐标。

## 任务五

`run_follow_trial.py`支持BAG（27/31/33）与BOTTLE（44）配置。默认DRY，LIVE需要预检、控制权和新鲜输入。每轮使用唯一run-id，输出目录由`--outdir`明确指定。

```bash
python3 run_follow_trial.py --mode dry --profile BAG --variant S5 \
  --seconds 5 --far-speed 22 --outdir /tmp/follow-runs --run-id bag_dry_01
```

## 任务六

`run_red_bottle.py`默认只计算控制。`--live`连接运动中继，`--follow`控制是否持续跟随；搜索、颜色、距离和速度参数通过`--help`查看，不能直接沿用任务五门槛。

```bash
python3 run_red_bottle.py --seconds 5 --url http://127.0.0.1:8080
```

统一上位机入口为[web/app.py](../web/app.py)，页面与HTTP服务说明见[web目录](../web/README.md)。本目录原有`webctl.py`保留为各策略快照的兼容入口；新部署优先使用统一网页入口。`b_live.sh`会重启网页服务，不作为首次部署的推荐入口。

## 观测与协议

- 视觉观测：类别、置信度、框中心/宽高、red_ratio、frame_id、producer_id与age。
- 控制命令：FPGA协议帧和CRC-8，编码由`cmd_relay.py`、`webctl.py`等封装。
- 遥测：轮速、前距、航向、任务状态及模式信息；控制逻辑须检查有效性和年龄。

完整数据流见[架构说明](../docs/architecture.md)。
