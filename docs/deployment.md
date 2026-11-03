# RK3588上位机部署

## 一体化入口

统一网页服务位于`web/`，以当前板端`http://10.36.218.39:8080`的源代码为依据。默认从仓库相对路径寻找任务六运行目录、检测器、模型和采集脚本，完整说明见[web/README](../web/README.md)。

## 环境准备

RK3588 aarch64 Linux、Python 3.9+、V4L2、GStreamer、g++/make及匹配的RKNN Runtime。模型和标签按[模型说明](../models/README.md)准备。`Pillow`用于检测BMP到MJPEG的可选转换；未提供时部分检测流预览能力受限。

```bash
cd software/detector
make abi
make
cd ../..
chmod +x software/runtime/grab_loop2.sh software/runtime/vision_ctl.sh
python3 web/app.py --check
```

模型、检测器和设备在离线文件检查之外，还需在实际RK环境验证。

## 手动启动

已有carweb服务运行时先确认并停用旧入口，或为调试实例配置独立端口与设备；不能同时让两套服务操作同一串口。

```bash
CMD_DEV=/dev/ttyS0 TLM_DEV=/dev/ttyUSB0 SPEED_MAX=24 python3 web/app.py
```

浏览器访问`http://<RK_IP>:8080`。网页与FPGA的任务三/四通过UART模式请求及遥测确认连接；任务五/六通过同仓库的策略进程执行。默认运行依赖是`software/task6_red_bottle`，与当前在线服务的三份关键策略文件哈希匹配。

## 配置与开机服务

环境变量示例：[carweb.env.example](../web/config/carweb.env.example)。支持命令/遥测串口、HTTP/TCP端口、速度上限和检测器/模型/任务目录的路径覆盖。

systemd模板：[robot-console.service.example](../web/config/robot-console.service.example)，示例安装位置为`/opt/fpga-rk3588-embodied-robot`。部署到其他目录时同步修改WorkingDirectory与ExecStart。首次切换前检查原carweb及drop-in配置，避免两个服务同时启用。模板本身不会自动安装或改变在线服务。

## 原有兼容入口

`software/task5_follow/webctl.py`、`software/task6_red_bottle/webctl.py`保留原策略快照的嵌入式网页实现及运行路径，供已有调用和回归使用。新的清晰上交结构采用`web/app.py`；不要混合两套控制权接口或并行启动。

## 视频与遥测

检测器与普通GStreamer预览都可能独占`/dev/video0`，由同一网页服务启停。页面刷新不代表新数据；检查`tlm_ok`、`tlm_age_ms`、距离、轮速和视觉frame_id/age。重启服务可能结束其检测子进程，之后需重新启用并确认帧推进。

2026-11-03已将新副本部署到RK，并通过静态复核，详见[板端复核](board_validation.md)。当前设备使用carweb的新增drop-in指向`/root/robot_submission_20261103_r03/fpga-rk3588-embodied-robot/web/app.py`，原`/root/task6/webctl.py`和基础服务文件未改；原有SPEED_MAX=60配置沿用。本轮未开启LIVE跟随。

如需回到该设备的旧入口，在停车并确认电机断电后，将本轮新增的`/etc/systemd/system/carweb.service.d/95-submission20261103.conf`移出drop-in目录，再`systemctl daemon-reload && systemctl restart carweb`。只处理这一项，不删除其他服务配置。新设备按模板安装，无需套用现场绝对路径。
