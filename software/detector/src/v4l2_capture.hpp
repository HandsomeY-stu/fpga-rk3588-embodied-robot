/*
 * v4l2_capture.hpp
 * V4L2内存映射采集封装，管理缓冲队列、视频帧与设备生命周期。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef V4L2_CAPTURE_HPP
#define V4L2_CAPTURE_HPP

#include <linux/videodev2.h>
#include <stdint.h>
#include <string>
#include <vector>

class V4L2Capture {
public:
    V4L2Capture();
    ~V4L2Capture();

    bool open(const std::string &dev, int width, int height, int buf_count = 4);

    bool reopen();

    bool grabFrame(uint8_t **data, size_t *bytes, int timeout_ms = 2000);
    void releaseFrame();
    void close();

    int width()  const { return m_width; }
    int height() const { return m_height; }
    uint32_t v4l2Format() const { return m_format; }
    uint32_t bytesPerLine() const { return m_bytesperline; }
    size_t frameBytes() const { return m_frame_bytes; }
    bool isOpen() const { return m_fd >= 0; }
    const std::string &lastError() const { return m_err; }
    const std::string &device() const { return m_dev; }    

private:
    struct Buf {
        void  *start = nullptr;
        size_t length = 0;
    };

    int         m_fd = -1;
    int         m_width = 0;
    int         m_height = 0;
    uint32_t    m_format = 0;
    uint32_t    m_bytesperline = 0;
    size_t      m_frame_bytes = 0;
    std::vector<Buf> m_bufs;
    int         m_cur_index = -1;
    bool        m_streaming = false;
    std::string m_err;

    std::string m_dev;
    int         m_req_w = 0;
    int         m_req_h = 0;
    int         m_buf_count = 4;
};

#endif  
