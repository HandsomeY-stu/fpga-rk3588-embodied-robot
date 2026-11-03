# 代码索引

任务五/六的同名模块属于独立策略快照，在各自目录启动。原厂IP生成内容保留，文件作用在此说明。

| 文件 | 作用 |
|---|---|
| [fpga/src/avoid/avoid_fsm.v](../fpga/src/avoid/avoid_fsm.v) | 超声波避障状态机，协调减速、停车和绕障动作。 |
| [fpga/src/avoid/ultrasonic_hcsr04.v](../fpga/src/avoid/ultrasonic_hcsr04.v) | HC-SR04触发与回波计时接口，完成距离转换和超时处理。 |
| [fpga/src/avoid/us_poll.v](../fpga/src/avoid/us_poll.v) | 多路超声波轮询调度，错开触发以降低通道相互干扰。 |
| [fpga/src/camera_init/camera_init.v](../fpga/src/camera_init/camera_init.v) | 摄像头上电及寄存器配置状态机，向I2C接口顺序发送初始化表。 |
| [fpga/src/camera_init/i2c_control/i2c_bit_shift.v](../fpga/src/camera_init/i2c_control/i2c_bit_shift.v) | I2C位级时序发生器，执行起始、数据、应答和停止序列。 |
| [fpga/src/camera_init/i2c_control/i2c_bit_shift_tb.v](../fpga/src/camera_init/i2c_control/i2c_bit_shift_tb.v) | RTL测试平台：验证i2c_bit_shift的事务和时序。 |
| [fpga/src/camera_init/i2c_control/i2c_control.v](../fpga/src/camera_init/i2c_control/i2c_control.v) | I2C读写事务控制器，封装寄存器访问与位级传输。 |
| [fpga/src/camera_init/i2c_control/i2c_control_tb.v](../fpga/src/camera_init/i2c_control/i2c_control_tb.v) | RTL测试平台：验证i2c_control的事务和时序。 |
| [fpga/src/camera_init/i2c_control/M24LC04B.v](../fpga/src/camera_init/i2c_control/M24LC04B.v) | I2C EEPROM仿真模型，用于验证I2C控制器。 |
| [fpga/src/camera_init/i2c_control/M24LC64.v](../fpga/src/camera_init/i2c_control/M24LC64.v) | I2C EEPROM仿真模型，用于验证I2C控制器。 |
| [fpga/src/camera_init/ov5640_init_table_jpeg.v](../fpga/src/camera_init/ov5640_init_table_jpeg.v) | OV5640 JPEG模式寄存器表；作为摄像头配置的备用模式保留。 |
| [fpga/src/camera_init/ov5640_init_table_rgb.v](../fpga/src/camera_init/ov5640_init_table_rgb.v) | OV5640 RGB模式寄存器表，配置分辨率、时序和自动曝光/白平衡。 |
| [fpga/src/camera_init/ov7725_init_table_rgb.v](../fpga/src/camera_init/ov7725_init_table_rgb.v) | OV7725备用寄存器表；本交付默认摄像头为OV5640。 |
| [fpga/src/camera_pll/camera_pll.v](../fpga/src/camera_pll/camera_pll.v) | Gowin生成的camera_pll IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/camera_pll/camera_pll_mod.v](../fpga/src/camera_pll/camera_pll_mod.v) | Gowin生成的camera_pll_mod IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/camera_pll/camera_pll_mod_tmp.v](../fpga/src/camera_pll/camera_pll_mod_tmp.v) | 厂商IP例化模板：camera_pll_mod；供重新生成IP或接口核对使用。 |
| [fpga/src/camera_pll/camera_pll_tmp.v](../fpga/src/camera_pll/camera_pll_tmp.v) | 厂商IP例化模板：camera_pll；供重新生成IP或接口核对使用。 |
| [fpga/src/comm/car_cmd_parse.v](../fpga/src/comm/car_cmd_parse.v) | A5/5A协议帧解析：检查长度和CRC并发布完整命令。 |
| [fpga/src/comm/uart_tx_8n1.v](../fpga/src/comm/uart_tx_8n1.v) | 参数化UART发送器，采用8数据位、无校验、1停止位。 |
| [fpga/src/comm/vis_osd_text.v](../fpga/src/comm/vis_osd_text.v) | 视觉检测结果转字符文本，为显示叠加提供目标信息。 |
| [fpga/src/comm/vis_uart_bridge.v](../fpga/src/comm/vis_uart_bridge.v) | 远程视觉/控制命令分发，处理模式请求、重复命令和回包。 |
| [fpga/src/comm/vision_follow_ctrl.v](../fpga/src/comm/vision_follow_ctrl.v) | FPGA视觉跟随接口控制器；与RK侧高级策略和命令安全门配合。 |
| [fpga/src/common/car_crc8.v](../fpga/src/common/car_crc8.v) | CRC-8校验模块，统一串口协议的数据完整性检查。 |
| [fpga/src/common/car_defines.vh](../fpga/src/common/car_defines.vh) | car_defines模块的实现或辅助入口，所属功能见代码索引。 |
| [fpga/src/common/sync_2ff.v](../fpga/src/common/sync_2ff.v) | 单比特双触发器同步器，用于跨时钟域状态采样。 |
| [fpga/src/ddr3_ctrl_2port/ddr3_ctrl_2port.v](../fpga/src/ddr3_ctrl_2port/ddr3_ctrl_2port.v) | 摄像头写入与显示读出双端口调度，连接FIFO和DDR3存储接口。 |
| [fpga/src/ddr3_ctrl_2port/fifo_ddr3_adapter.v](../fpga/src/ddr3_ctrl_2port/fifo_ddr3_adapter.v) | FIFO侧与DDR3突发访问侧之间的数据/握手适配。 |
| [fpga/src/ddr3_memory_interface/ddr3_memory_interface.v](../fpga/src/ddr3_memory_interface/ddr3_memory_interface.v) | Gowin生成的ddr3_memory_interface IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/ddr3_memory_interface/ddr3_memory_interface_tmp.v](../fpga/src/ddr3_memory_interface/ddr3_memory_interface_tmp.v) | 厂商IP例化模板：ddr3_memory_interface；供重新生成IP或接口核对使用。 |
| [fpga/src/ddr_pll/ddr_pll.v](../fpga/src/ddr_pll/ddr_pll.v) | Gowin生成的ddr_pll IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/ddr_pll/ddr_pll_mod.v](../fpga/src/ddr_pll/ddr_pll_mod.v) | Gowin生成的ddr_pll_mod IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/ddr_pll/ddr_pll_mod_tmp.v](../fpga/src/ddr_pll/ddr_pll_mod_tmp.v) | 厂商IP例化模板：ddr_pll_mod；供重新生成IP或接口核对使用。 |
| [fpga/src/ddr_pll/ddr_pll_tmp.v](../fpga/src/ddr_pll/ddr_pll_tmp.v) | 厂商IP例化模板：ddr_pll；供重新生成IP或接口核对使用。 |
| [fpga/src/despeckle/despeckle_median5.v](../fpga/src/despeckle/despeckle_median5.v) | 水平五点中值滤波；分级比较网络与像素有效信号配合，抑制孤立采集噪点。 |
| [fpga/src/disp/hc595_driver.v](../fpga/src/disp/hc595_driver.v) | 74HC595串行移位及锁存驱动。 |
| [fpga/src/disp/hex8.v](../fpga/src/disp/hex8.v) | 八位数码管扫描与显示编码。 |
| [fpga/src/dvi_tx/dvi_tx.v](../fpga/src/dvi_tx/dvi_tx.v) | Gowin生成的dvi_tx IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/dvi_tx/dvi_tx_tmp.v](../fpga/src/dvi_tx/dvi_tx_tmp.v) | 厂商IP例化模板：dvi_tx；供重新生成IP或接口核对使用。 |
| [fpga/src/DVP_Capture/DVP_Capture.v](../fpga/src/DVP_Capture/DVP_Capture.v) | DVP摄像头数据采集：按有效窗口组合RGB565字节并生成写缓存请求。 |
| [fpga/src/gowin_ddr/gowin_ddr.v](../fpga/src/gowin_ddr/gowin_ddr.v) | Gowin生成的gowin_ddr IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/gowin_ddr/gowin_ddr_tmp.v](../fpga/src/gowin_ddr/gowin_ddr_tmp.v) | 厂商IP例化模板：gowin_ddr；供重新生成IP或接口核对使用。 |
| [fpga/src/imu/angle_text_fmt.v](../fpga/src/imu/angle_text_fmt.v) | 将姿态角转换为定长字符文本，使用双缓冲发布完整文本。 |
| [fpga/src/imu/att_align.v](../fpga/src/imu/att_align.v) | IMU坐标系到车体坐标系的姿态映射。 |
| [fpga/src/imu/jy901_rx.v](../fpga/src/imu/jy901_rx.v) | JY901姿态串口帧解析，校验帧头/校验和并发布新姿态。 |
| [fpga/src/motion/car_ctrl_top.v](../fpga/src/motion/car_ctrl_top.v) | 运动子系统集成：命令解码、模式仲裁、避障/导航接管及状态输出。 |
| [fpga/src/motion/encoder_quad.v](../fpga/src/motion/encoder_quad.v) | 正交编码器采样与方向判定，输出带符号脉冲计数。 |
| [fpga/src/motion/motion_ctrl.v](../fpga/src/motion/motion_ctrl.v) | 编码器速度闭环、电机目标速度分配与里程反馈，支持直行和差速转向。 |
| [fpga/src/motion/motion_safety_gate.v](../fpga/src/motion/motion_safety_gate.v) | 运动命令安全门：检查急停、命令时效和控制权限，输出最终放行信号。 |
| [fpga/src/motion/motor_drive.v](../fpga/src/motion/motor_drive.v) | 左右电机方向和PWM驱动封装，连接闭环控制输出与物理引脚。 |
| [fpga/src/motion/pid_ctrl.v](../fpga/src/motion/pid_ctrl.v) | 定点PID速度调节器，按离散控制周期更新并对输出限幅。 |
| [fpga/src/motion/pwm_gen.v](../fpga/src/motion/pwm_gen.v) | 可参数化PWM发生器，将占空比目标转换为周期性输出。 |
| [fpga/src/nav/nav_fsm.v](../fpga/src/nav/nav_fsm.v) | 通用路点导航引擎；源码保留，当前集成默认采用任务四固定路线模式。 |
| [fpga/src/nav/odom_xy.v](../fpga/src/nav/odom_xy.v) | 利用轮速增量和航向估计平面位姿，采用定点积分。 |
| [fpga/src/nav/sincos_lut.v](../fpga/src/nav/sincos_lut.v) | 角度到正余弦的定点查找表，为里程投影提供系数。 |
| [fpga/src/osd/osd_mode_cdc.v](../fpga/src/osd/osd_mode_cdc.v) | OSD模式的跨时钟握手：稳定数据保持至应答，显示模式在帧边界提交。 |
| [fpga/src/osd/osd_overlay.v](../fpga/src/osd/osd_overlay.v) | 像素流OSD字符叠加，生成姿态角等文字的显示区域。 |
| [fpga/src/ov5640_ddr3_hdmi.v](../fpga/src/ov5640_ddr3_hdmi.v) | FPGA系统顶层：连接摄像头、DDR3帧缓存、HDMI、IMU、运动控制及远程命令。 |
| [fpga/src/pll_init.v](../fpga/src/pll_init.v) | PLL初始化控制，配合生成的时钟IP完成启动。 |
| [fpga/src/pll_mDRP_intf.v](../fpga/src/pll_mDRP_intf.v) | PLL动态重配置接口，保留原工程时钟控制实现。 |
| [fpga/src/rd_data_fifo/rd_data_fifo.v](../fpga/src/rd_data_fifo/rd_data_fifo.v) | Gowin生成的rd_data_fifo IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/rd_data_fifo/rd_data_fifo_tmp.v](../fpga/src/rd_data_fifo/rd_data_fifo_tmp.v) | 厂商IP例化模板：rd_data_fifo；供重新生成IP或接口核对使用。 |
| [fpga/src/rgb_neutral_trim.v](../fpga/src/rgb_neutral_trim.v) | RGB565到RGB888的中性色彩补偿；使用查找表完成舍入和饱和运算。 |
| [fpga/src/task4/task4_seg1_fsm.v](../fpga/src/task4/task4_seg1_fsm.v) | 任务四固定路线状态机：双障碍绕行、累计里程和v48尾段三次左转。 |
| [fpga/src/vga/disp_driver.v](../fpga/src/vga/disp_driver.v) | HDMI显示时序发生及RGB输出，生成像素请求和帧边界。 |
| [fpga/src/vga/disp_parameter_cfg.v](../fpga/src/vga/disp_parameter_cfg.v) | 显示分辨率和消隐时序的参数定义。 |
| [fpga/src/vga/dvi_encoder.v](../fpga/src/vga/dvi_encoder.v) | RGB与同步信号的TMDS编码及串行输出。 |
| [fpga/src/wr_data_fifo/wr_data_fifo.v](../fpga/src/wr_data_fifo/wr_data_fifo.v) | Gowin生成的wr_data_fifo IP或封装；保持生成内容与工具接口一致。 |
| [fpga/src/wr_data_fifo/wr_data_fifo_tmp.v](../fpga/src/wr_data_fifo/wr_data_fifo_tmp.v) | 厂商IP例化模板：wr_data_fifo；供重新生成IP或接口核对使用。 |
| [scripts/check_repository.py](../scripts/check_repository.py) | 离线检查Python语法、工程引用、C++本地依赖和固件哈希。 |
| [scripts/run_python_tests.py](../scripts/run_python_tests.py) | 运行策略与本地模拟服务回归，不访问开发板。 |
| [scripts/run_rtl.py](../scripts/run_rtl.py) | 在临时目录编译执行RTL回归，支持可选厂商原语库。 |
| [sim/rtl/tb_osd_mode_cdc.v](../sim/rtl/tb_osd_mode_cdc.v) | RTL测试平台：验证osd_mode_cdc的接口、边界条件或算法等价性。 |
| [sim/rtl/tb_remote34.v](../sim/rtl/tb_remote34.v) | RTL测试平台：验证remote34的接口、边界条件或算法等价性。 |
| [sim/rtl/tb_rgb_neutral_trim.v](../sim/rtl/tb_rgb_neutral_trim.v) | RTL测试平台：验证rgb_neutral_trim的接口、边界条件或算法等价性。 |
| [sim/rtl/tb_task4_controls.v](../sim/rtl/tb_task4_controls.v) | RTL测试平台：验证task4_controls的接口、边界条件或算法等价性。 |
| [sim/rtl/tb_task4_seg1_tail.v](../sim/rtl/tb_task4_seg1_tail.v) | RTL测试平台：验证task4_seg1_tail的接口、边界条件或算法等价性。 |
| [sim/rtl/tb_trim_equiv.v](../sim/rtl/tb_trim_equiv.v) | RTL测试平台：验证trim_equiv的接口、边界条件或算法等价性。 |
| [software/detector/include/rknn_api.h](../software/detector/include/rknn_api.h) | 适配librknnrt 1.6.x的最小RKNN C API声明；配套ABI检查核对结构布局。 |
| [software/detector/include/vision_types.hpp](../software/detector/include/vision_types.hpp) | 视觉观测、目标框和跟踪状态的公共数据结构。 |
| [software/detector/scripts/abi_check.cpp](../software/detector/scripts/abi_check.cpp) | RKNN结构体ABI布局自检，检查大小与字段偏移。 |
| [software/detector/scripts/test_yolo.cpp](../software/detector/scripts/test_yolo.cpp) | 实际C++后处理单元测试，不访问摄像头或电机。 |
| [software/detector/sim/dry_run_command_sink.hpp](../software/detector/sim/dry_run_command_sink.hpp) | 离线命令接收器，记录运动请求而不访问串口。 |
| [software/detector/src/centroid_tracker.hpp](../software/detector/src/centroid_tracker.hpp) | 轻量质心目标跟踪，维护目标关联、速度估计与丢失期限。 |
| [software/detector/src/realtime_ssd.cpp](../software/detector/src/realtime_ssd.cpp) | 实时检测主程序：V4L2常驻采集、RKNN推理、YOLO后处理及红色区域统计；文件名沿用早期接口。 |
| [software/detector/src/realtime_ssd_jsonl.cpp](../software/detector/src/realtime_ssd_jsonl.cpp) | 检测JSONL输出实现，保留逐帧观测的离线接口。 |
| [software/detector/src/ssd_postprocess.cpp](../software/detector/src/ssd_postprocess.cpp) | 检测数据结构与SSD兼容后处理，供实时推理主程序共同使用。 |
| [software/detector/src/ssd_postprocess.hpp](../software/detector/src/ssd_postprocess.hpp) | 检测数据结构与SSD兼容后处理，供实时推理主程序共同使用。 |
| [software/detector/src/target_selector.hpp](../software/detector/src/target_selector.hpp) | 目标选择器，按类别、可靠性与身份连续性选出控制目标。 |
| [software/detector/src/task12_motion_bridge.hpp](../software/detector/src/task12_motion_bridge.hpp) | 视觉观测到运动控制接口的桥接层，管理目标有效性与命令转换。 |
| [software/detector/src/v4l2_capture.cpp](../software/detector/src/v4l2_capture.cpp) | V4L2内存映射采集封装，管理缓冲队列、视频帧与设备生命周期。 |
| [software/detector/src/v4l2_capture.hpp](../software/detector/src/v4l2_capture.hpp) | V4L2内存映射采集封装，管理缓冲队列、视频帧与设备生命周期。 |
| [software/detector/src/yolo_postprocess.hpp](../software/detector/src/yolo_postprocess.hpp) | YOLO检测解码、坐标映射及非极大值抑制，输出统一目标框。 |
| [software/detector/uart/uart_control.hpp](../software/detector/uart/uart_control.hpp) | 串口运动控制协议常量、帧编码和接口封装。 |
| [software/detector/uart/uart_vision.hpp](../software/detector/uart/uart_vision.hpp) | 视觉目标UART帧编码与收发，使用统一CRC和明确的无目标表示。 |
| [software/fpga_web_modes.py](../software/fpga_web_modes.py) | 网页请求到FPGA任务模式的映射，集中管理模式切换接口。 |
| [software/task5_follow/actuator_probe.py](../software/task5_follow/actuator_probe.py) | 诊断辅助程序：采集actuator相关接口响应。 |
| [software/task5_follow/beep_test.py](../software/task5_follow/beep_test.py) | beep_test模块的实现或辅助入口，所属功能见代码索引。 |
| [software/task5_follow/calib.py](../software/task5_follow/calib.py) | 摄像头中心与运动响应的标定辅助工具。 |
| [software/task5_follow/cmd_relay.py](../software/task5_follow/cmd_relay.py) | TCP到串口运动命令中继，处理序号与命令时效。 |
| [software/task5_follow/dir_probe.py](../software/task5_follow/dir_probe.py) | 诊断辅助程序：采集dir相关接口响应。 |
| [software/task5_follow/follow_policy.py](../software/task5_follow/follow_policy.py) | 任务五跟随策略：目标身份、可靠性判定、速度整形和最终保护按统一顺序执行。 |
| [software/task5_follow/fpga_web_modes.py](../software/task5_follow/fpga_web_modes.py) | 网页请求到FPGA任务模式的映射，集中管理模式切换接口。 |
| [software/task5_follow/frame_pub.py](../software/task5_follow/frame_pub.py) | 将采集环形缓冲中的完整图像原子发布为网页预览帧。 |
| [software/task5_follow/measure_vision_gate.py](../software/task5_follow/measure_vision_gate.py) | 静态视觉门槛检查，统计目标命中率、帧率与数据新鲜度。 |
| [software/task5_follow/nav_fsm_sw.py](../software/task5_follow/nav_fsm_sw.py) | 软件导航状态机，描述导航阶段及完成/中止条件。 |
| [software/task5_follow/nav_layer.py](../software/task5_follow/nav_layer.py) | 导航任务的策略封装，连接观测、阶段切换及运动输出。 |
| [software/task5_follow/prop_ctrl.py](../software/task5_follow/prop_ctrl.py) | 图像横向误差和距离的跟随控制器，提供迟滞纠偏、速度曲线和丢失处理。 |
| [software/task5_follow/prop_offline_test.py](../software/task5_follow/prop_offline_test.py) | 离线回归测试：检查prop_offline_test相关逻辑和异常处理。 |
| [software/task5_follow/rec_boxes.py](../software/task5_follow/rec_boxes.py) | 检测框录制工具，用于离线检查目标连续性。 |
| [software/task5_follow/red_bottle_target.py](../software/task5_follow/red_bottle_target.py) | 目标关联与红色目标策略，综合语义类别、颜色比例、空间连续性和丢失计时。 |
| [software/task5_follow/red_nav_e2e.py](../software/task5_follow/red_nav_e2e.py) | 红色目标导航流程的端到端执行与结果记录。 |
| [software/task5_follow/run_b.py](../software/task5_follow/run_b.py) | 兼容跟随执行入口，保留既有控制器的命令行接口。 |
| [software/task5_follow/run_c.py](../software/task5_follow/run_c.py) | 连续跟随执行入口，连接视觉、遥测和差速控制。 |
| [software/task5_follow/run_follow_trial.py](../software/task5_follow/run_follow_trial.py) | 任务五执行入口，提供DRY/LIVE模式、预检、控制权管理和结束停车。 |
| [software/task5_follow/run_red_bottle.py](../software/task5_follow/run_red_bottle.py) | 任务六执行入口，读取红瓶识别与遥测，计算并按需发送靠近命令。 |
| [software/task5_follow/scan_probe.py](../software/task5_follow/scan_probe.py) | 诊断辅助程序：采集scan相关接口响应。 |
| [software/task5_follow/semantic_nav.py](../software/task5_follow/semantic_nav.py) | 视觉目标搜索与语义任务编排，协调方位、目标状态与运动命令。 |
| [software/task5_follow/speed_probe.py](../software/task5_follow/speed_probe.py) | 诊断辅助程序：采集speed相关接口响应。 |
| [software/task5_follow/stage1_regression.py](../software/task5_follow/stage1_regression.py) | 跟随策略阶段回归，复用离线输入检查控制输出。 |
| [software/task5_follow/telemetry_log.py](../software/task5_follow/telemetry_log.py) | FPGA遥测解码与记录工具，提取轮速、距离和航向等字段。 |
| [software/task5_follow/test_continuity.py](../software/task5_follow/test_continuity.py) | 离线回归测试：检查continuity相关逻辑和异常处理。 |
| [software/task5_follow/test_follow_policy.py](../software/task5_follow/test_follow_policy.py) | 离线回归测试：检查follow_policy相关逻辑和异常处理。 |
| [software/task5_follow/test_qualify_phase.py](../software/task5_follow/test_qualify_phase.py) | 离线回归测试：检查qualify_phase相关逻辑和异常处理。 |
| [software/task5_follow/test_runner.py](../software/task5_follow/test_runner.py) | 离线回归测试：检查runner相关逻辑和异常处理。 |
| [software/task5_follow/test_scan_mock.py](../software/task5_follow/test_scan_mock.py) | 离线回归测试：检查scan_mock相关逻辑和异常处理。 |
| [software/task5_follow/test_speedup.py](../software/task5_follow/test_speedup.py) | 离线回归测试：检查speedup相关逻辑和异常处理。 |
| [software/task5_follow/test_stall_mock.py](../software/task5_follow/test_stall_mock.py) | 离线回归测试：检查stall_mock相关逻辑和异常处理。 |
| [software/task5_follow/test_support.py](../software/task5_follow/test_support.py) | 离线回归测试：检查support相关逻辑和异常处理。 |
| [software/task5_follow/webctl.py](../software/task5_follow/webctl.py) | RK侧网页控制与TCP中继服务，管理遥测、检测进程及任务启动/停止。 |
| [software/task5_follow/yaw_turn.py](../software/task5_follow/yaw_turn.py) | 基于航向误差的有界转向辅助程序。 |
| [software/task6_red_bottle/actuator_probe.py](../software/task6_red_bottle/actuator_probe.py) | 诊断辅助程序：采集actuator相关接口响应。 |
| [software/task6_red_bottle/beep_test.py](../software/task6_red_bottle/beep_test.py) | beep_test模块的实现或辅助入口，所属功能见代码索引。 |
| [software/task6_red_bottle/calib.py](../software/task6_red_bottle/calib.py) | 摄像头中心与运动响应的标定辅助工具。 |
| [software/task6_red_bottle/cmd_relay.py](../software/task6_red_bottle/cmd_relay.py) | TCP到串口运动命令中继，处理序号与命令时效。 |
| [software/task6_red_bottle/dir_probe.py](../software/task6_red_bottle/dir_probe.py) | 诊断辅助程序：采集dir相关接口响应。 |
| [software/task6_red_bottle/fpga_web_modes.py](../software/task6_red_bottle/fpga_web_modes.py) | 网页请求到FPGA任务模式的映射，集中管理模式切换接口。 |
| [software/task6_red_bottle/lookaround.py](../software/task6_red_bottle/lookaround.py) | 有界视野搜索辅助程序，按阶段调整观察方向。 |
| [software/task6_red_bottle/nav_fsm_sw.py](../software/task6_red_bottle/nav_fsm_sw.py) | 软件导航状态机，描述导航阶段及完成/中止条件。 |
| [software/task6_red_bottle/nav_layer.py](../software/task6_red_bottle/nav_layer.py) | 导航任务的策略封装，连接观测、阶段切换及运动输出。 |
| [software/task6_red_bottle/prop_ctrl.py](../software/task6_red_bottle/prop_ctrl.py) | 图像横向误差和距离的跟随控制器，提供迟滞纠偏、速度曲线和丢失处理。 |
| [software/task6_red_bottle/red_bottle_target.py](../software/task6_red_bottle/red_bottle_target.py) | 目标关联与红色目标策略，综合语义类别、颜色比例、空间连续性和丢失计时。 |
| [software/task6_red_bottle/red_nav_e2e.py](../software/task6_red_bottle/red_nav_e2e.py) | 红色目标导航流程的端到端执行与结果记录。 |
| [software/task6_red_bottle/run_b.py](../software/task6_red_bottle/run_b.py) | 兼容跟随执行入口，保留既有控制器的命令行接口。 |
| [software/task6_red_bottle/run_c.py](../software/task6_red_bottle/run_c.py) | 连续跟随执行入口，连接视觉、遥测和差速控制。 |
| [software/task6_red_bottle/run_red_bottle.py](../software/task6_red_bottle/run_red_bottle.py) | 任务六执行入口，读取红瓶识别与遥测，计算并按需发送靠近命令。 |
| [software/task6_red_bottle/scan_probe.py](../software/task6_red_bottle/scan_probe.py) | 诊断辅助程序：采集scan相关接口响应。 |
| [software/task6_red_bottle/semantic_nav.py](../software/task6_red_bottle/semantic_nav.py) | 视觉目标搜索与语义任务编排，协调方位、目标状态与运动命令。 |
| [software/task6_red_bottle/speed_probe.py](../software/task6_red_bottle/speed_probe.py) | 诊断辅助程序：采集speed相关接口响应。 |
| [software/task6_red_bottle/telemetry_log.py](../software/task6_red_bottle/telemetry_log.py) | FPGA遥测解码与记录工具，提取轮速、距离和航向等字段。 |
| [software/task6_red_bottle/test_stall_mock.py](../software/task6_red_bottle/test_stall_mock.py) | 离线回归测试：检查stall_mock相关逻辑和异常处理。 |
| [software/task6_red_bottle/webctl.py](../software/task6_red_bottle/webctl.py) | RK侧网页控制与TCP中继服务，管理遥测、检测进程及任务启动/停止。 |
| [software/task6_red_bottle/yaw_turn.py](../software/task6_red_bottle/yaw_turn.py) | 基于航向误差的有界转向辅助程序。 |

## 上位机网页

| 文件 | 作用 |
|---|---|
| [web/app.py](../web/app.py) | 统一网页启动与离线文件检查入口。 |
| [web/server.py](../web/server.py) | 当前上位机HTTP接口、TCP中继、遥测与任务进程管理。 |
| [web/assets.py](../web/assets.py) | HTML加载与静态资源白名单。 |
| [web/fpga_web_modes.py](../web/fpga_web_modes.py) | FPGA模式请求、停止与遥测确认。 |
| [web/test_web.py](../web/test_web.py) | 串口模拟和本机回环页面/API回归。 |
| [web/static/index.html](../web/static/index.html) | 网页任务控制、视频和遥测结构。 |
| [web/static/style.css](../web/static/style.css) | 上位机响应式布局和状态样式。 |
| [web/static/app.js](../web/static/app.js) | 遥控心跳、模式/任务交互与状态刷新。 |

## 交付校验与离线复盘

模型资产清单见[models/assets_manifest.json](../models/assets_manifest.json)，固件哈希与报告见[firmware/README](../firmware/README.md)。七个离线工具的用途和参数见[离线重放](replay.md)。
