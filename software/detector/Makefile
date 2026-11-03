# RK3588常驻目标检测器：构建YOLO程序，并提供CPU后处理与ABI离线检查。
CXX      ?= g++
CXXFLAGS  = -O2 -std=c++11 -Wall -Iinclude -Isrc -Iuart
LDFLAGS   = -L/usr/lib -lrknnrt -lpthread -ldl -lrt

TARGET    = realtime_yolo
SRCS      = src/realtime_ssd.cpp src/v4l2_capture.cpp src/ssd_postprocess.cpp
OBJS      = $(SRCS:.cpp=.o)

DEMO      ?= /root/rknn_ssd_demo_Linux
MODEL     ?= ../../models/yolov5s-640-640.rknn
PRIORS    ?= ../../models/box_priors.txt
LABELS    ?= ../../models/coco_labels_list.txt
ROADIMG   ?= $(DEMO)/model/road.bmp
DEVICE    ?= /dev/video0
SECS      ?= 600
CONF      ?= 0.40
NMS       ?= 0.45

.PHONY: all clean run stab img check

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS)
	@echo "=== 编译完成: $(TARGET) ==="
	@ls -l $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

check:
	@echo "--- librknnrt ---"; ls -l /usr/lib/librknnrt.so || echo "缺少 librknnrt!"
	@echo "--- 关键符号 ---"; nm -D /usr/lib/librknnrt.so | grep -wE 'rknn_init|rknn_query|rknn_run' || echo "符号缺失!"
	@echo "--- 视频设备 ---"; ls -l /dev/video0 || echo "无 /dev/video0 (HDMI-IN)"
	@echo "--- g++ ---"; $(CXX) --version | head -1

# 单图回归：验证后处理链路（应与 demo 一致出 9 框）
img: $(TARGET)
	./$(TARGET) -m $(MODEL) -a $(PRIORS) -l $(LABELS) \
		--image $(ROADIMG) --conf $(CONF) --nms $(NMS) \
		--dump logs/road_annotated.bmp

run: $(TARGET)
	./$(TARGET) -m $(MODEL) -a $(PRIORS) -l $(LABELS) -d $(DEVICE) \
		--secs $(SECS) --conf $(CONF) --nms $(NMS)

# 10 分钟稳定性验收
stab: $(TARGET)
	@mkdir -p logs
	./$(TARGET) -m $(MODEL) -a $(PRIORS) -l $(LABELS) -d $(DEVICE) \
		--secs 600 --conf $(CONF) --nms $(NMS) \
		--dump logs/dump_stab.bmp 2>&1 | tee logs/stability_600s.log

abi:
	@echo "=== RKNN ABI 布局自校验 ==="
	$(CXX) -std=c++17 -Wall -Iinclude -Isrc -Iuart scripts/abi_check.cpp -o abi_check
	@./abi_check

clean:
	rm -f $(OBJS) $(TARGET) abi_check test_yolo

.PHONY: all clean run check img abi stab

# Actual C++ postprocessor unit tests, not RTL or camera acceptance.
test: scripts/test_yolo.cpp src/yolo_postprocess.hpp
	$(CXX) $(CXXFLAGS) scripts/test_yolo.cpp -o test_yolo
	./test_yolo

