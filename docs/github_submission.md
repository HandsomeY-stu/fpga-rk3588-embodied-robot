# 上传GitHub

建议仓库名：`fpga-rk3588-embodied-robot`。将本目录的内容作为仓库根，GitHub打开后即可显示README。

## 用Git上传

在本目录执行，远程地址替换为自己的仓库地址：

```bash
git init -b main
git add .
git commit -m "Submit FPGA and RK3588 robot system"
git remote add origin <YOUR_REPOSITORY_URL>
git push -u origin main
```

仓库已提供`.gitignore`，构建产物、运行日志、其他训练/转换权重、缓存和本地配置不会进入提交。**三个已验证运行资产已随包包含**，其中`models/yolov5s-640-640.rknn`不会被忽略；保留模型来源与许可。`firmware/`包含0x325C烧录FS、源码/哈希清单、资源及详细时序报告和验证摘要，上传时一起保留。

## 上传前检查

```bash
python scripts/check_repository.py
git status --short
```

README没有填写未知的团队名称、人员、奖项或演示视频。需要时可自行补充实物图、演示链接和团队信息，实际测试状态按验证说明填写。

GitHub Actions配置用于源码、RTL和离线单元检查，首次push后查看实际运行结果；配置文件存在不表示远端CI已经运行通过。

## 本次最新版内容

完整FPGA源码与约束、RK3588检测/控制源码、独立网页前后端、模型/标签/priors、FS、构建说明和测试脚本均已纳入。不含内部交接日志、现场原始图像、私钥、密码或运行日志。用户已补充反馈其余功能全部验证正确，实测来源与自动化证据见验证文档。

不要把ZIP文件本身上传成唯一仓库文件；先解压，将内层目录中的`README.md`、`fpga/`、`software/`、`web/`等直接作为仓库根内容。上传操作由团队在自己的目标仓库执行，本次仅交付文件。
