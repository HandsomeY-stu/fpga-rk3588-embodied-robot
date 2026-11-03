# 上位机网页与服务

本目录整合用户当前上位机网址`http://10.36.218.39:8080`对应的网页代码。地址是本次设备配置，换网络后使用实际RK3588地址。

原工程将HTML、CSS和JavaScript嵌入`webctl.py`。本目录从**当前在线服务快照**整理页面与接口，并拆出静态资源，作为总工程清晰可见的上位机入口。

| 文件 | 作用 |
|---|---|
| `app.py` | 统一启动入口；`--check`只检查文件，不打开串口 |
| `server.py` | HTTP API、TCP中继、遥测接收、识别进程与任务控制 |
| `static/index.html` | 遥控、任务开关、遥测与视频区域的页面结构 |
| `static/style.css` | 卡片布局、状态灯、按钮和移动端样式 |
| `static/app.js` | 按键心跳、状态轮询、视频刷新和任务/API交互 |
| `fpga_web_modes.py` | 手动、FPGA避障和任务四模式请求及遥测确认 |
| `assets.py` | 固定白名单静态资源加载，拒绝任意路径读取 |
| `config/` | 环境变量与systemd启动模板 |
| `test_web.py` | 本机模拟串口的页面/API回归，不连接开发板 |

## 控制与数据流

浏览器 → HTTP API → 命令仲裁/任务进程 → FPGA命令串口。

FPGA遥测 → 网页服务状态 → 浏览器轮询；检测图/预览帧 → 图片接口 → 视频区域。

页面保留手动遥控、停车/急停、任务三FPGA避障、任务四自主导航、任务五指定目标跟随、任务六靠近红瓶、姿态OSD和识别预览。任务四实际固定路线由随包v48位流执行，通用路点引擎仍默认关闭。

## 快速检查

在仓库根运行：

```bash
python web/app.py --check
python web/test_web.py
```

第二条测试只启动本机回环HTTP服务，串口被模拟，不启动发送/遥测/采集线程。

## 在RK3588启动

先按[模型说明](../models/README.md)准备模型、标签并构建检测器。确保旧carweb服务已停用或选择其他端口，不能让两套服务同时控制串口。

```bash
chmod +x software/runtime/grab_loop2.sh software/runtime/vision_ctl.sh
CMD_DEV=/dev/ttyS0 TLM_DEV=/dev/ttyUSB0 SPEED_MAX=24 python3 web/app.py
```

浏览器访问`http://<RK_IP>:8080`。命令/遥测串口可用环境变量覆盖，检测器、模型和采集脚本默认从仓库相对路径定位；示例配置位于`config/`。

后台靠近/跟随入口默认使用`software/task6_red_bottle/run_red_bottle.py`及同目录策略，与本次在线服务依赖的文件哈希一致。`software/task5_follow/run_follow_trial.py`仍提供任务五独立DRY/LIVE试验入口，其控制权接口属于该试验快照，不应未经核对就与本网页同时运行。

## 常用接口

| 方法/路径 | 用途 |
|---|---|
| GET `/` | 上位机页面 |
| GET `/api/status` | 模式、距离、航向、轮速、新鲜度及任务状态 |
| GET `/api/frame.jpg` | 普通预览 |
| GET `/api/person.bmp`、`/api/person.json` | 检测图和同帧检测数据 |
| GET `/api/task6_status` | 红瓶控制器状态 |
| POST `/api/motion` | 手动方向与速度请求 |
| POST `/api/stop`、`/api/estop` | 停车、急停 |
| POST `/api/fpga_mode` | 手动0、FPGA避障1、任务四4 |
| POST `/api/person` | 检测器启停 |
| POST `/api/task6` | 跟随或红瓶靠近任务 |
| POST `/api/target`、`/api/osd` | 目标配置、OSD0/1切换 |
| POST `/api/preview`、`/api/guard` | 预览与软件守护接口兼容 |

服务模板用于新副本部署。2026-11-03已在RK新目录实际部署并验证，原源码保留；识别/独占预览期间不重启普通采集链路。当前API版本标记为`integrated-v48-20261103-r03`。结果见[板端复核](../docs/board_validation.md)。
