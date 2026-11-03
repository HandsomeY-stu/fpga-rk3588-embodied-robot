# 构建与验证

## FPGA

- 工具：Gowin EDA V1.9.12 x64。
- 器件：GW5AT-LV60PG484AC1/I0（ACG720）。
- 工程：`fpga/ov5640_ddr3_hdmi.gprj`。
- 顶层：`gao_ov5640_ddr_hdmi`。

```bash
gw_sh fpga/build.tcl
```

Windows可使用EDA安装目录的`gw_sh.exe`。脚本从自身位置解析工程路径，include目录为`src/common`。构建输出在`fpga/impl/`，该目录不入版本控制。

约束与生成IP属于构建必要输入，完整保留。每次重新构建后检查资源、详细setup/hold/recovery/removal报告及各时钟Fmax；综合成功不等于时序收敛。不要使用全局false_path隐藏像素或控制数据路径。

当前目标：camera_pclk 56MHz、控制50MHz、显示75MHz；摄像头采集1280×720/20fps。HDMI重复读帧的显示刷新率与摄像头采集帧率不同。

## RKNN检测器

在aarch64 Linux / RK3588上安装g++、make，并提供匹配的`librknnrt.so`。

```bash
cd software/detector
make abi
make
make test
```

`include/rknn_api.h`为项目使用的最小声明，`make abi`核对该声明的关键布局。升级Runtime后须重新检查ABI与模型兼容性。

原包漏带的头文件和测试源已从已有工程补齐。`TARGET=realtime_yolo`，主源文件仍名为`realtime_ssd.cpp`以保留原接口，不代表实际采用SSD模型。

## 离线回归

```bash
python scripts/check_repository.py
python scripts/run_python_tests.py
python scripts/run_rtl.py
```

RTL工具可通过`IVERILOG`、`VVP`指定；RGB原语测试通过`GOWIN_SIMLIB`指定Gowin的`prim_sim.v`。未安装库时默认只跳过RGB项，结果中明确显示（公共CI通常为5组+RGB跳过）。需要严格运行六组时使用`python scripts/run_rtl.py --require-gowin`，缺库会失败。

## 发布位流

随包位流UserCode为0x325C，SHA256为：

```text
205046b2936cde17a770f291c3819977b2f951901ccf9222402300de8d5b0827
```

仓库注释整理不改变源代码逻辑，随包`firmware/source_manifest.json`记录源码绑定与校验信息。重新生成的位流应另存为新版本，不用旧UserCode和哈希代替新构建验证。
