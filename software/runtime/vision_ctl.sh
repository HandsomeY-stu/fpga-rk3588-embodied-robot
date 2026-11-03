#!/bin/bash
# 板端图像采集链路控制，支持普通预览、录像和状态查询。
set -u

kill_gst() {
    pkill -INT -x gst-launch-1.0 2>/dev/null
    sleep 1
    pkill -9 -x gst-launch-1.0 2>/dev/null
    sleep 0.5
}

kill_loop() {
    # 用 pgrep 取 PID 再 kill，避免模式串匹配到自身
    for p in $(pgrep -f "bin/bash /root/grab_loop2.sh" 2>/dev/null) \
             $(pgrep -f "^/root/grab_loop2.sh" 2>/dev/null); do
        [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null
    done
    sleep 0.3
}

case "${1:-}" in
plain)
    rm -f /tmp/rec_on
    kill_gst; kill_loop
    setsid /root/grab_loop2.sh > /tmp/grab2.log 2>&1 < /dev/null &
    sleep 4
    ls -l --time-style=+%H:%M:%S /tmp/fr/ 2>/dev/null | tail -2
    ;;
rec)
    OUT="${2:-/tmp/rec.mp4}"
    rm -f "$OUT"
    echo "$OUT" > /tmp/rec_on
    kill_gst; kill_loop
    setsid /root/grab_loop2.sh > /tmp/grab2.log 2>&1 < /dev/null &
    sleep 5
    ls -l "$OUT" 2>/dev/null || echo "（录像文件尚未出现，mp4 可能要几秒）"
    ;;
stop)
    kill_gst
    rm -f /tmp/rec_on
    # 交回普通抓帧（看门狗 5s 内会自己起来；这里也主动起一次）
    setsid /root/grab_loop2.sh > /tmp/grab2.log 2>&1 < /dev/null &
    sleep 4
    ls -l --time-style=+%H:%M:%S /tmp/fr/ 2>/dev/null | tail -2
    ;;
status)
    echo -n "录像标记: "; [ -s /tmp/rec_on ] && cat /tmp/rec_on || echo 关闭
    echo -n "gst 进程: "; pgrep -x gst-launch-1.0 >/dev/null && echo 在跑 || echo 没跑
    echo -n "抓帧脚本: "; pgrep -f "grab_loop2" >/dev/null && echo 在跑 || echo 没跑
    echo -n "网页帧: "; ls -l --time-style=+%H:%M:%S /tmp/frame.jpg 2>/dev/null | awk '{print $6, $5"B"}'
    ls -l --time-style=+%H:%M:%S /tmp/rec*.mp4 2>/dev/null | awk '{print "录像: "$6, $7, $5"B"}'
    date +"现在: %H:%M:%S"
    ;;
*)
    echo "用法: $0 {plain|rec <file>|stop|status}"; exit 1;;
esac
