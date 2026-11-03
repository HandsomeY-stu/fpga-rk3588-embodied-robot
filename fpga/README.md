# FPGA工程

入口：[ov5640_ddr3_hdmi.gprj](ov5640_ddr3_hdmi.gprj)，顶层`gao_ov5640_ddr_hdmi`。命令行构建使用[build.tcl](build.tcl)。

| 子目录 | 功能 |
|---|---|
| DVP_Capture / camera_init / despeckle | 摄像头配置、采集与流水滤波 |
| ddr3_ctrl_2port / ddr3_memory_interface / *_fifo | 帧缓存与读写适配 |
| imu / osd / vga | 姿态接收、文字叠加、显示时序 |
| motion | 编码器、PID、PWM、模式仲裁与安全门 |
| avoid | 超声波与实时避障 |
| task4 / nav | 固定路线状态机及备用通用导航模块 |
| comm / common | 协议解析、UART、CRC及跨域同步 |

摄像头保持1280×720/20fps、BYTE_SWAP=1、自动曝光/增益/白平衡；行翻转及颜色修复配置沿用集成版。默认姿态OSD保留，任务四采用v48固定路线。

厂商加密IP、接口封装与参数文件保留原内容；每个文件的用途见[代码索引](../docs/code_map.md)。详细约束与验证见[构建说明](../docs/build.md)和[验证说明](../docs/validation.md)。
