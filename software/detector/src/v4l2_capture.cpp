/*
 * v4l2_capture.cpp
 * V4L2内存映射采集封装，管理缓冲队列、视频帧与设备生命周期。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#include "v4l2_capture.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#define NPLANES 1

static std::string fourcc_str(uint32_t f) {
    char s[5];
    s[0] = (char)( f        & 0xFF);
    s[1] = (char)((f >> 8)  & 0xFF);
    s[2] = (char)((f >> 16) & 0xFF);
    s[3] = (char)((f >> 24) & 0xFF);
    s[4] = '\0';
    return std::string(s);
}

V4L2Capture::V4L2Capture() {}
V4L2Capture::~V4L2Capture() { close(); }

bool V4L2Capture::open(const std::string &dev, int width, int height, int buf_count) {
    close();
    m_err.clear();

    m_dev       = dev;
    m_req_w     = width;
    m_req_h     = height;
    m_buf_count = buf_count;

    m_fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (m_fd < 0) {
        m_err = "open(" + dev + ") failed: " + strerror(errno);
        return false;
    }

    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    bool mplane = true;
    if (ioctl(m_fd, VIDIOC_QUERYCAP, &cap) == 0) {
        printf("[v4l2] driver=%s card=%s caps=0x%08x\n",
               cap.driver, cap.card, cap.capabilities);
        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
            mplane = true;
        else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
            mplane = false;
        printf("[v4l2] 设备类型: %s\n", mplane ? "Multiplanar" : "Single-planar");
    }
    if (!mplane) {
        m_err = "本实现只支持 multiplanar 设备（rk_hdmirx 即为此类）";
        close();
        return false;
    }

    const enum v4l2_buf_type buftype = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(m_fd, VIDIOC_G_FMT, &fmt) == 0) {
        m_width        = fmt.fmt.pix_mp.width;
        m_height       = fmt.fmt.pix_mp.height;
        m_format       = fmt.fmt.pix_mp.pixelformat;
        m_bytesperline = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
        printf("[v4l2] G_FMT: %dx%d fourcc=%s bpl=%u sizeimage=%u num_planes=%u\n",
               m_width, m_height, fourcc_str(m_format).c_str(),
               m_bytesperline, fmt.fmt.pix_mp.plane_fmt[0].sizeimage,
               fmt.fmt.pix_mp.num_planes);
    } else {
        printf("[v4l2] G_FMT 失败(%s)，用请求值 %dx%d\n", strerror(errno), width, height);
        m_width  = width;
        m_height = height;
    }

    struct v4l2_format want;
    memset(&want, 0, sizeof(want));
    want.type = buftype;
    want.fmt.pix_mp.width       = m_width;
    want.fmt.pix_mp.height      = m_height;
    want.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_BGR24;
    want.fmt.pix_mp.field       = V4L2_FIELD_NONE;
    want.fmt.pix_mp.num_planes  = NPLANES;
    want.fmt.pix_mp.plane_fmt[0].bytesperline = 0;
    want.fmt.pix_mp.plane_fmt[0].sizeimage    = (uint32_t)m_width * m_height * 3;
    if (ioctl(m_fd, VIDIOC_S_FMT, &want) == 0) {
        m_width        = want.fmt.pix_mp.width;
        m_height       = want.fmt.pix_mp.height;
        m_format       = want.fmt.pix_mp.pixelformat;
        m_bytesperline = want.fmt.pix_mp.plane_fmt[0].bytesperline;
        printf("[v4l2] S_FMT 后: %dx%d fourcc=%s bpl=%u\n",
               m_width, m_height, fourcc_str(m_format).c_str(), m_bytesperline);
    } else {
        printf("[v4l2] S_FMT 不被接受(%s)，沿用驱动格式\n", strerror(errno));
    }

    if (m_bytesperline == 0) m_bytesperline = (uint32_t)m_width * 3;
    m_frame_bytes = (size_t)m_bytesperline * m_height;

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = buf_count;
    req.type   = buftype;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(m_fd, VIDIOC_REQBUFS, &req) < 0) {
        m_err = std::string("VIDIOC_REQBUFS failed: ") + strerror(errno);
        close();
        return false;
    }
    if (req.count < 2) {
        m_err = "驱动返回 buffer 数不足 2";
        close();
        return false;
    }
    printf("[v4l2] REQBUFS 得到 %u 个 buffer\n", req.count);

    m_bufs.clear();
    for (uint32_t i = 0; i < req.count; i++) {
        struct v4l2_buffer b;
        struct v4l2_plane  planes[NPLANES];
        memset(&b, 0, sizeof(b));
        memset(planes, 0, sizeof(planes));
        b.type    = buftype;
        b.memory  = V4L2_MEMORY_MMAP;
        b.index   = i;
        b.m.planes = planes;
        b.length   = NPLANES;

        if (ioctl(m_fd, VIDIOC_QUERYBUF, &b) < 0) {
            m_err = std::string("VIDIOC_QUERYBUF failed: ") + strerror(errno);
            close();
            return false;
        }
        Buf nb;
        nb.length = planes[0].length;
        nb.start  = mmap(NULL, planes[0].length, PROT_READ | PROT_WRITE,
                         MAP_SHARED, m_fd, planes[0].m.mem_offset);
        if (nb.start == MAP_FAILED) {
            m_err = std::string("mmap failed: ") + strerror(errno);
            close();
            return false;
        }
        m_bufs.push_back(nb);
        printf("[v4l2] buf[%u] mmap len=%zu offset=0x%llx\n",
               i, nb.length, (unsigned long long)planes[0].m.mem_offset);

        if (ioctl(m_fd, VIDIOC_QBUF, &b) < 0) {
            m_err = std::string("VIDIOC_QBUF(init) failed: ") + strerror(errno);
            close();
            return false;
        }
    }

    enum v4l2_buf_type type = buftype;
    if (ioctl(m_fd, VIDIOC_STREAMON, &type) < 0) {
        m_err = std::string("VIDIOC_STREAMON failed: ") + strerror(errno);
        close();
        return false;
    }
    m_streaming = true;

    printf("[v4l2] 已开启 %s: %dx%d %s bpl=%u, %zu bufs\n",
           dev.c_str(), m_width, m_height, fourcc_str(m_format).c_str(),
           m_bytesperline, m_bufs.size());
    return true;
}

bool V4L2Capture::grabFrame(uint8_t **data, size_t *bytes, int timeout_ms) {
    if (m_fd < 0 || !m_streaming) return false;
    if (m_cur_index >= 0) releaseFrame();

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_fd, &fds);
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int r = select(m_fd + 1, &fds, NULL, NULL, &tv);
    if (r < 0) {
        if (errno == EINTR) return false;
        m_err = std::string("select failed: ") + strerror(errno);
        return false;
    }
    if (r == 0) { m_err = "select timeout"; return false; }

    struct v4l2_buffer b;
    struct v4l2_plane  planes[NPLANES];
    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.memory   = V4L2_MEMORY_MMAP;
    b.m.planes = planes;
    b.length   = NPLANES;

    if (ioctl(m_fd, VIDIOC_DQBUF, &b) < 0) {
        if (errno == EAGAIN) { m_err = "DQBUF EAGAIN"; return false; }
        m_err = std::string("VIDIOC_DQBUF failed: ") + strerror(errno);
        return false;
    }
    m_cur_index = (int)b.index;
    *data  = (uint8_t *)m_bufs[b.index].start;
    *bytes = planes[0].bytesused ? planes[0].bytesused : m_frame_bytes;
    return true;
}

void V4L2Capture::releaseFrame() {
    if (m_fd < 0 || m_cur_index < 0) return;
    struct v4l2_buffer b;
    struct v4l2_plane  planes[NPLANES];
    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.memory   = V4L2_MEMORY_MMAP;
    b.index    = (uint32_t)m_cur_index;
    b.m.planes = planes;
    b.length   = NPLANES;
    ioctl(m_fd, VIDIOC_QBUF, &b);
    m_cur_index = -1;
}

bool V4L2Capture::reopen() {
    if (m_dev.empty() || m_req_w <= 0 || m_req_h <= 0) {
        m_err = "reopen: 无历史 open 配置（需先成功 open 过一次）";
        return false;
    }

    const std::string dev = m_dev;
    const int w = m_req_w, h = m_req_h, n = m_buf_count;
    printf("[v4l2] reopen：重建 %s %dx%d bufs=%d\n", dev.c_str(), w, h, n);
    return open(dev, w, h, n);    
}

void V4L2Capture::close() {
    if (m_fd >= 0 && m_streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(m_fd, VIDIOC_STREAMOFF, &type);
        m_streaming = false;
    }

    for (auto &b : m_bufs)
        if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    m_bufs.clear();

    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
    m_streaming = false;
    m_cur_index = -1;
}
