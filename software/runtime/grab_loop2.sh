#!/bin/bash
# 网页采集循环，生成环形JPEG缓冲并可选录制视频。
mkdir -p /tmp/fr

if [ ! -f /tmp/fr/f_000.jpg ]; then rm -f /tmp/fr/*.jpg; fi

while true; do
  if [ -s /tmp/rec_on ]; then
    OUT=$(cat /tmp/rec_on)
    # 同一目标文件已存在（例如 gst 崩溃重启）⇒ 加时间戳，绝不覆盖已录内容
    if [ -s "$OUT" ]; then
      OUT="${OUT%.mp4}_$(date +%H%M%S).mp4"
    fi
    echo "$(date +%H:%M:%S) [grab] 录像模式 → $OUT"
    gst-launch-1.0 -e v4l2src device=/dev/video0 \
      ! video/x-raw,width=1280,height=720 \
      ! tee name=t \
      t. ! queue max-size-buffers=3 leaky=downstream \
         ! videoconvert ! videoscale ! video/x-raw,width=320,height=180 \
         ! jpegenc quality=75 \
         ! multifilesink location=/tmp/fr/f_%03d.jpg max-files=3 \
      t. ! queue max-size-buffers=8 leaky=downstream \
         ! mpph264enc bps=6000000 \
         ! h264parse config-interval=1 \
         ! mp4mux \
         ! filesink location="$OUT"
    echo "$(date +%H:%M:%S) [grab] 录像管道退出（code=$?），1 秒后重开"
  else
    gst-launch-1.0 -q v4l2src device=/dev/video0 \
      ! videoconvert \
      ! videoscale \
      ! video/x-raw,width=320,height=180 \
      ! jpegenc quality=75 \
      ! multifilesink location=/tmp/fr/f_%03d.jpg max-files=3
    echo "$(date +%H:%M:%S) gst 退出（code=$?），1 秒后重开"
  fi
  sleep 1
done
