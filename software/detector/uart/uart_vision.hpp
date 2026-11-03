/*
 * uart_vision.hpp
 * 视觉目标UART帧编码与收发，使用统一CRC和明确的无目标表示。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef UART_VISION_HPP
#define UART_VISION_HPP

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>

namespace uartvis {

static const uint8_t FRAME_HEAD_H = 0xA5;
static const uint8_t FRAME_HEAD_L = 0x5A;
static const uint8_t CMD_PING     = 0x7E;
static const uint8_t CMD_VISION   = 0x20;
static const uint8_t TLM_PONG     = 0x8F;
static const uint8_t TLM_VISION   = 0x86;

static const int    ECHO_TIMEOUT_MS = 300;

static const int    FINISH_WAIT_MS  = 450;

static const size_t RX_ACC_MAX      = 4096;

static const double ECHO_STALE_MS   = 2000.0;

struct Stats {
    unsigned long tx_ok;           
    unsigned long tx_err;          
    unsigned long tx_short;        
    unsigned long valid_sent;      
    unsigned long invalid_sent;    
    unsigned long echo_ok;         
    unsigned long echo_mismatch;   
    unsigned long echo_crc_err;    
    unsigned long echo_timeout;    
    unsigned long teardown_sent;   
    unsigned long teardown_err;    
    int           ever_echo_ok;    
    double        last_echo_ok_t;  
};

struct State {
    int         fd;
    uint8_t     seq;
    int         inited;
    std::vector<uint8_t> acc;

    struct Pending {
        bool      used;
        uint8_t   cls, conf;
        uint16_t  cx, cy, w, h;
        double    t_sent;
    };
    Pending pend[256];

    Stats st;
};

inline State& state() {
    static State s;
    return s;
}

inline Stats& stats() { return state().st; }

inline double now_sec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

inline uint8_t crc8_byte(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; ++i)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

inline std::vector<uint8_t> build_frame(uint8_t seq, uint8_t cmd,
                                        const std::vector<uint8_t>& data) {
    uint8_t len = (uint8_t)(1 + data.size());      
    std::vector<uint8_t> f;
    f.reserve((size_t)len + 5);
    f.push_back(FRAME_HEAD_H);
    f.push_back(FRAME_HEAD_L);
    f.push_back(len);
    f.push_back(seq);
    f.push_back(cmd);
    uint8_t c = crc8_byte(0x00, len);
    c = crc8_byte(c, seq);
    c = crc8_byte(c, cmd);
    for (size_t i = 0; i < data.size(); ++i) { f.push_back(data[i]); c = crc8_byte(c, data[i]); }
    f.push_back(c);
    return f;
}

inline int open_serial(const char* dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("[uart] open"); return -1; }
    termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) { perror("[uart] tcgetattr"); close(fd); return -1; }
    cfmakeraw(&tty);
    speed_t sp = B115200;
    if (baud == 9600)   sp = B9600;
    if (baud == 57600)  sp = B57600;
    if (baud == 230400) sp = B230400;
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("[uart] tcsetattr"); close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

inline int read_some(int fd, uint8_t* buf, int max, int timeout_ms) {
    fd_set rf; FD_ZERO(&rf); FD_SET(fd, &rf);
    timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int r = select(fd + 1, &rf, NULL, NULL, &tv);
    if (r <= 0) return 0;
    int n = (int)read(fd, buf, max);
    return n > 0 ? n : 0;
}

inline bool parse_frame(std::vector<uint8_t>& acc, uint8_t& seq, uint8_t& cmd,
                        std::vector<uint8_t>& data, int* crc_errs = NULL) {
    size_t i = 0;
    while (i + 5 <= acc.size()) {
        if (acc[i] != FRAME_HEAD_H || acc[i + 1] != FRAME_HEAD_L) { ++i; continue; }
        uint8_t len = acc[i + 2];
        if (len < 1 || len > 32) { ++i; continue; }
        size_t need = (size_t)5 + (len - 1) + 1;    
        if (i + need > acc.size()) break;           

        uint8_t s = acc[i + 3], c = acc[i + 4];
        uint8_t crc = crc8_byte(0x00, len);
        crc = crc8_byte(crc, s);
        crc = crc8_byte(crc, c);
        std::vector<uint8_t> d;
        for (size_t k = 0; k < (size_t)(len - 1); ++k) {
            uint8_t b = acc[i + 5 + k];
            d.push_back(b);
            crc = crc8_byte(crc, b);
        }
        uint8_t got = acc[i + need - 1];
        if (crc != got) {
            if (crc_errs) (*crc_errs)++;
            ++i;                                    
            continue;
        }
        seq = s; cmd = c; data = d;
        acc.erase(acc.begin(), acc.begin() + i + need);
        return true;
    }
    if (i > 0) acc.erase(acc.begin(), acc.begin() + i);
    if (acc.size() > RX_ACC_MAX) acc.erase(acc.begin(), acc.begin() + (acc.size() - RX_ACC_MAX));
    return false;
}

inline size_t pending_count() {
    State& s = state();
    size_t n = 0;
    for (int i = 0; i < 256; ++i) if (s.pend[i].used) ++n;
    return n;
}

inline void last_echo_age_str(char* out, int n) {
    State& s = state();
    if (!s.st.ever_echo_ok) { snprintf(out, (size_t)n, "unknown"); return; }
    double age = (now_sec() - s.st.last_echo_ok_t) * 1000.0;
    if (age > ECHO_STALE_MS) snprintf(out, (size_t)n, "stale(%.0fms)", age);
    else                     snprintf(out, (size_t)n, "%.0fms", age);
}

inline bool vis_uart_init(const char* dev = "/dev/ttyS0", int baud = 115200) {
    State& s = state();
    if (s.inited && s.fd >= 0) return true;
    memset(&s.pend, 0, sizeof(s.pend));
    memset(&s.st, 0, sizeof(s.st));
    s.seq = 0;
    s.acc.clear();
    s.fd = open_serial(dev, baud);
    s.inited = 1;
    if (s.fd < 0) {
        fprintf(stderr, "[uart] 打开 %s 失败：UART 发送已禁用（图像采集/推理不受影响）\n", dev);
        return false;
    }
    printf("[uart] 已打开 %s @ %d 8N1（CMD_VISION 发送就绪）\n", dev, baud);
    return true;
}

inline bool vis_uart_ready() {
    State& s = state();
    return s.inited && s.fd >= 0;
}

inline void vis_drain_echo() {
    State& s = state();
    if (!vis_uart_ready()) return;

    for (;;) {
        uint8_t buf[512];
        int n = read_some(s.fd, buf, (int)sizeof(buf), 0);
        if (n <= 0) break;
        s.acc.insert(s.acc.end(), buf, buf + n);
    }

    for (;;) {
        uint8_t seq = 0, cmd = 0;
        std::vector<uint8_t> d;
        int crc_n = 0;
        bool got = parse_frame(s.acc, seq, cmd, d, &crc_n);
        if (crc_n) s.st.echo_crc_err += (unsigned long)crc_n;
        if (!got) break;
        if (cmd != TLM_VISION) continue;              
        State::Pending& p = s.pend[seq];
        if (!p.used) continue;                         
        if (d.size() >= 10) {
            uint16_t cx = (uint16_t)(d[1] | (d[2] << 8));
            uint16_t cy = (uint16_t)(d[3] | (d[4] << 8));
            uint16_t w  = (uint16_t)(d[5] | (d[6] << 8));
            uint16_t h  = (uint16_t)(d[7] | (d[8] << 8));
            bool ok = (d[0] == p.cls && cx == p.cx && cy == p.cy &&
                       w == p.w && h == p.h && d[9] == p.conf);
            if (ok) {
                s.st.echo_ok++;
                s.st.ever_echo_ok = 1;
                s.st.last_echo_ok_t = now_sec();
            } else {
                s.st.echo_mismatch++;
            }
        } else {
            s.st.echo_mismatch++;
        }
        p.used = false;
    }

    double t = now_sec();
    for (int i = 0; i < 256; ++i) {
        State::Pending& p = s.pend[i];
        if (p.used && (t - p.t_sent) * 1000.0 > ECHO_TIMEOUT_MS) {
            p.used = false;
            s.st.echo_timeout++;
        }
    }
}

inline void vis_send_raw(uint8_t cls, uint16_t cx, uint16_t cy,
                         uint16_t w, uint16_t h, uint8_t conf) {
    State& s = state();
    if (!vis_uart_ready()) return;

    vis_drain_echo();                  

    uint8_t seq = s.seq++;
    std::vector<uint8_t> data;
    data.reserve(10);
    data.push_back(cls);
    data.push_back((uint8_t)(cx & 0xFF)); data.push_back((uint8_t)(cx >> 8));
    data.push_back((uint8_t)(cy & 0xFF)); data.push_back((uint8_t)(cy >> 8));
    data.push_back((uint8_t)(w  & 0xFF)); data.push_back((uint8_t)(w  >> 8));
    data.push_back((uint8_t)(h  & 0xFF)); data.push_back((uint8_t)(h  >> 8));
    data.push_back(conf);

    std::vector<uint8_t> f = build_frame(seq, CMD_VISION, data);

    ssize_t n = write(s.fd, f.data(), f.size());
    if (n < 0) {
        s.st.tx_err++;
        return;                                    
    }
    if ((size_t)n != f.size()) {
        s.st.tx_short++;
        return;                                    
    }
    s.st.tx_ok++;

    State::Pending& p = s.pend[seq];
    p.used = true;
    p.cls = cls; p.cx = cx; p.cy = cy; p.w = w; p.h = h; p.conf = conf;
    p.t_sent = now_sec();
}

inline void vis_send_box(int cls, int cx, int cy, int w, int h, int conf) {
    if (cls < 0)    cls = 0;
    if (cls > 255)  cls = 255;
    if (cx < 0)     cx = 0;
    if (cx > 65535) cx = 65535;
    if (cy < 0)     cy = 0;
    if (cy > 65535) cy = 65535;
    if (w  < 0)     w  = 0;
    if (w  > 65535) w  = 65535;
    if (h  < 0)     h  = 0;
    if (h  > 65535) h  = 65535;
    if (conf < 0)   conf = 0;
    if (conf > 100) conf = 100;
    vis_send_raw((uint8_t)cls, (uint16_t)cx, (uint16_t)cy,
                 (uint16_t)w, (uint16_t)h, (uint8_t)conf);
    stats().valid_sent++;
}

inline void vis_send_invalid() {
    vis_send_raw(0, 0, 0, 0, 0, 0);
    stats().invalid_sent++;
}

inline void vis_send_teardown_invalid() {
    State& s = state();
    if (!vis_uart_ready()) return;
    uint8_t seq = s.seq++;
    std::vector<uint8_t> data(10, 0);
    std::vector<uint8_t> f = build_frame(seq, CMD_VISION, data);
    ssize_t n = write(s.fd, f.data(), f.size());
    s.st.teardown_sent++;
    if (n < 0 || (size_t)n != f.size()) s.st.teardown_err++;
}

inline void vis_print_stats(const char* tag = "统计") {
    State& s = state();
    const Stats& us = s.st;
    size_t pend = pending_count();
    unsigned long lhs = us.tx_ok;
    unsigned long rhs = us.echo_ok + us.echo_mismatch + us.echo_timeout + (unsigned long)pend;
    char age[64]; last_echo_age_str(age, (int)sizeof(age));

    printf(" ---- UART 视觉帧统计 [%s] (CMD_VISION 0x20 / TLM_VISION 0x86) ----\n", tag);
    printf(" 发送 成功/失败/短写 : %lu / %lu / %lu\n", us.tx_ok, us.tx_err, us.tx_short);
    printf(" 帧数 有效框/无效态  : %lu / %lu\n", us.valid_sent, us.invalid_sent);
    printf(" 回包 匹配/不符/CRC错/超时 : %lu / %lu / %lu / %lu  pending=%zu\n",
           us.echo_ok, us.echo_mismatch, us.echo_crc_err, us.echo_timeout, pend);
    printf(" 最近匹配回包年龄    : %s\n", age);
    printf(" teardown 帧 发送/失败: %lu / %lu（不计入业务口径）\n",
           us.teardown_sent, us.teardown_err);
    printf(" 不变量 tx_ok(%lu) == echo_ok+mismatch+timeout+pending(%lu) : %s\n",
           lhs, rhs, (lhs == rhs) ? "PASS" : "FAIL");
}

inline void vis_uart_finish(int max_wait_ms = FINISH_WAIT_MS, bool send_teardown = true) {
    State& s = state();
    if (!s.inited) return;

    if (send_teardown) vis_send_teardown_invalid();

    if (vis_uart_ready()) {
        double t0 = now_sec();
        for (;;) {
            vis_drain_echo();
            if (pending_count() == 0) break;
            if ((now_sec() - t0) * 1000.0 >= (double)max_wait_ms) break;
            usleep(5000);
        }
        vis_drain_echo();
    }

    vis_print_stats("收尾");

    if (s.fd >= 0) {
        close(s.fd);
        s.fd = -1;
        printf("[uart] 串口已显式关闭（fd=-1，无残留占用）\n");
    }
}

inline int vis_poll_echo(int* cls, int* cx, int* cy, int* w, int* h, int* conf)
    __attribute__((unused));
inline int vis_poll_echo(int* cls, int* cx, int* cy, int* w, int* h, int* conf) {
    State& s = state();
    if (!vis_uart_ready()) return 0;
    for (;;) {
        uint8_t seq = 0, cmd = 0;
        std::vector<uint8_t> d;
        int crc_n = 0;
        bool got = parse_frame(s.acc, seq, cmd, d, &crc_n);
        if (crc_n) s.st.echo_crc_err += (unsigned long)crc_n;
        if (!got) return 0;
        if (cmd != TLM_VISION || d.size() < 10) continue;
        if (cls)  *cls  = d[0];
        if (cx)   *cx   = (int)(uint16_t)(d[1] | (d[2] << 8));
        if (cy)   *cy   = (int)(uint16_t)(d[3] | (d[4] << 8));
        if (w)    *w    = (int)(uint16_t)(d[5] | (d[6] << 8));
        if (h)    *h    = (int)(uint16_t)(d[7] | (d[8] << 8));
        if (conf) *conf = d[9];
        return 1;
    }
}

}   

#endif  
