/*
 * uart_control.hpp
 * 串口运动控制协议常量、帧编码和接口封装。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef UART_CONTROL_HPP
#define UART_CONTROL_HPP

#include <cstdint>
#include <vector>

namespace uartctl {

static const uint8_t FRAME_HEAD_H  = 0xA5;
static const uint8_t FRAME_HEAD_L  = 0x5A;
static const uint8_t CRC8_POLY     = 0x07;
static const uint8_t CRC8_INIT     = 0x00;
static const int     FRAME_LEN_MAX = 32;       

static const uint8_t CMD_MOTION    = 0x01;     
static const uint8_t CMD_SPEED     = 0x02;     
static const uint8_t CMD_MODE      = 0x03;     
static const uint8_t CMD_STOP      = 0x04;     
static const uint8_t CMD_ROTATE    = 0x05;     
static const uint8_t CMD_WAYPOINT  = 0x10;     
static const uint8_t CMD_NAV_START = 0x11;     
static const uint8_t CMD_NAV_ABORT = 0x12;     
static const uint8_t CMD_VISION    = 0x20;     
static const uint8_t CMD_VISION_EN = 0x21;     
static const uint8_t CMD_PID_SET   = 0x30;
static const uint8_t CMD_PING      = 0x7E;
static const uint8_t CMD_QUERY     = 0x7F;
static const uint8_t CMD_SOFT_RST  = 0xF0;

static const uint8_t TLM_VISION = 0x86;
static const uint8_t TLM_PONG   = 0x8F;
static const uint8_t TLM_STATUS = 0x84;
static const uint8_t TLM_FAULT  = 0x9F;

static const uint8_t DIR_STOP   = 0x00;
static const uint8_t DIR_FWD    = 0x01;
static const uint8_t DIR_BACK   = 0x02;
static const uint8_t DIR_LEFT   = 0x03;
static const uint8_t DIR_RIGHT  = 0x04;
static const uint8_t DIR_SPIN_L = 0x05;
static const uint8_t DIR_SPIN_R = 0x06;

static const uint8_t MODE_MANUAL = 0x00;
static const uint8_t MODE_AVOID  = 0x01;
static const uint8_t MODE_NAV    = 0x02;
static const uint8_t MODE_VISION = 0x03;

inline uint8_t crc8_byte(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; ++i)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRC8_POLY) : (uint8_t)(crc << 1);
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
    uint8_t c = crc8_byte(CRC8_INIT, len);
    c = crc8_byte(c, seq);
    c = crc8_byte(c, cmd);
    for (size_t i = 0; i < data.size(); ++i) { f.push_back(data[i]); c = crc8_byte(c, data[i]); }
    f.push_back(c);
    return f;
}

inline uint8_t frame_crc(const std::vector<uint8_t>& f) {

    uint8_t c = CRC8_INIT;
    for (size_t i = 2; i + 1 < f.size(); ++i) c = crc8_byte(c, f[i]);
    return c;
}

inline std::vector<uint8_t> motion_frame(uint8_t seq, uint8_t dir, uint8_t speed) {
    std::vector<uint8_t> d;
    d.push_back(dir); d.push_back(speed);
    return build_frame(seq, CMD_MOTION, d);
}

inline std::vector<uint8_t> stop_frame(uint8_t seq) {
    std::vector<uint8_t> d;
    return build_frame(seq, CMD_STOP, d);
}

inline std::vector<uint8_t> vision_frame(uint8_t seq, uint8_t cls,
                                         uint16_t cx, uint16_t cy,
                                         uint16_t w, uint16_t h, uint8_t conf) {
    std::vector<uint8_t> d;
    d.push_back(cls);
    d.push_back((uint8_t)(cx & 0xFF)); d.push_back((uint8_t)(cx >> 8));
    d.push_back((uint8_t)(cy & 0xFF)); d.push_back((uint8_t)(cy >> 8));
    d.push_back((uint8_t)(w  & 0xFF)); d.push_back((uint8_t)(w  >> 8));
    d.push_back((uint8_t)(h  & 0xFF)); d.push_back((uint8_t)(h  >> 8));
    d.push_back(conf);
    return build_frame(seq, CMD_VISION, d);
}

inline std::vector<uint8_t> vision_invalid_frame(uint8_t seq) {
    std::vector<uint8_t> d(10, 0);
    return build_frame(seq, CMD_VISION, d);
}

inline std::string hex(const std::vector<uint8_t>& f) {
    static const char* H = "0123456789ABCDEF";
    std::string s;
    s.reserve(f.size() * 2);
    for (size_t i = 0; i < f.size(); ++i) { s.push_back(H[f[i] >> 4]); s.push_back(H[f[i] & 0xF]); }
    return s;
}

}  

#endif  
