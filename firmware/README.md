# 固件、源码与构建报告

烧录文件：[integrated_t4v48_uc325c.fs](integrated_t4v48_uc325c.fs)，UserCode `0x0000325C`。

SHA256：`205046b2936cde17a770f291c3819977b2f951901ccf9222402300de8d5b0827`。

[source_manifest.json](source_manifest.json)区分两套路径均相对`fpga/`的哈希：`original_fpga_hashes`绑定生成该位流的原始构建输入；`submission_fpga_hashes`逐字节绑定当前带注释的交付文件。两者注释不同的文件不应被当作哈希失配；交付检查脚本核验当前文件使用后者。

`build.tcl`和`ov5640_ddr3_hdmi.gprj`位于`fpga/`根目录，清单中没有给它们添加`src/`前缀。整理后的源代码已在前轮做注释外等价检查，本轮未改动FPGA源文件、IP、约束或构建参数。

## 配套真实构建报告

来自该位流原始Gowin 1.9.12构建输出，非本轮重新布线：

- [资源/实现文本报告](reports/ov5640_ddr3_hdmi.rpt.txt)
- [资源/实现HTML报告](reports/ov5640_ddr3_hdmi.rpt.html)
- [详细时序页面](reports/ov5640_ddr3_hdmi_tr_content.html)
- [时序导航入口](reports/ov5640_ddr3_hdmi.tr.html)

各报告的SHA256见清单。控制域50MHz的Fmax为50.083MHz，余量小；各同域setup/hold TNS为0，不代表完整设计零违例。DDR IP及PLL/复位相关负路径仍保留在详细报告。

## 2026-11-03实际板端复核

[board_validation.json](board_validation.json)记录本轮摘要；说明见[验收与边界](../docs/board_validation.md)。从原交付ZIP提取这颗FS下载后，独立UserCode回读为0x325C，视频、OSD、遥测与模式接口验证通过。

Programmer操作4的最终结果仍是`Verify Failed`/退出码18，故未把SRAM逐位校验记为通过；功能通过基于独立回读和真实运行数据。本次仅下载SRAM，未写入非易失Flash，FPGA断电后需要重新下载。上述自动化阶段未进行电机上电行驶验收；随后用户反馈其余功能均验证正确，已独立记录在board_validation.json。
