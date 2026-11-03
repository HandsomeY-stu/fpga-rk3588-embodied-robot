# 模型与运行资产

本目录包含当前RK3588已使用的三个匹配资产，可直接用于随包检测器，无需先转换模型。2026-11-03与在线文件逐一核对SHA256一致。

| 文件 | 字节数 | SHA256 |
|---|---:|---|
| `yolov5s-640-640.rknn` | 8,477,315 | `d4ecf73883e7bea27d59dc7a521f6786c6ce6b3b146ed45ee7b6df0e84d80ad9` |
| `box_priors.txt` | 76,929 | `3560aa6b03bb7f69240b6efcac88dbf56ffa3c73b0da13f695bf97c2206d2089` |
| `coco_labels_list.txt` | 664 | `6d4b61aedabd129c30f2d5c0d8ed6da0068fb975a91e0085544792d6b3d3ca5e` |

机器可读清单：[assets_manifest.json](assets_manifest.json)。在仓库根运行`python3 scripts/check_repository.py`会核对三项的大小和SHA256。

## 模型来源与兼容条件

权重来自[Rockchip rknpu2固定提交的RK3588 YOLOv5示例](https://github.com/rockchip-linux/rknpu2/blob/5adf7c1bd17e169e9880ccdf3b49adde925ab7f9/examples/rknn_yolov5_demo/model/RK3588/yolov5s-640-640.rknn)。对应上游许可见[LICENSE.rknpu2](LICENSE.rknpu2)，保留Rockchip版权与免责声明；本项目未对该权重重新训练。

输入为640×640 RGB uint8，预处理等比缩放并填充114灰边；输出为三个NCHW检测头 `[1,255,80,80]`、`[1,255,40,40]`、`[1,255,20,20]`，与随包YOLO后处理匹配。80个稠密类别转为原协议的稀疏COCO编号，例如瓶子39→44。标签/priors继承板端已验证的Rockchip SSD示例加载接口，不把该priors用于YOLO框解码。

**`box_priors.txt`在当前YOLO启动路径中也必须存在**：程序先执行共享`loadAnchors()`，成功后才进入YOLO后处理。缺失会使程序退出；“YOLO不使用SSD priors解码”不等于可以删除此文件。

报告中`3e4bd54c963059a92091a69564add6de`是模型MD5，准确SHA256为表中的64位值。RKNN Runtime/驱动/SDK仍需安装到实际RK3588，动态库不随源码仓库分发；不能仅改模型文件名替换为另一种YOLO输出格式。

如果需要重新下载，按上述固定提交保存为`yolov5s-640-640.rknn`并运行清单检查。运行与构建步骤见[部署说明](../docs/deployment.md)。
