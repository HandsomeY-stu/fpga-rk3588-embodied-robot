/*
 * vision_types.hpp
 * 视觉观测、目标框和跟踪状态的公共数据结构。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef VISION_TYPES_HPP
#define VISION_TYPES_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace vision {

struct Detection {
    int      class_id;       
    std::string label;       
    float    confidence;     
    int      x, y, w, h;     
    int64_t  frame_seq;      
    int64_t  timestamp_ms;   
    float    color_score;    

    Detection() : class_id(-1), confidence(0.f), x(0), y(0), w(0), h(0),
                  frame_seq(0), timestamp_ms(0), color_score(-1.f) {}

    int cx() const { return x + w / 2; }
    int cy() const { return y + h / 2; }
    int area() const { return (w > 0 && h > 0) ? w * h : 0; }
};

struct FrameDetections {
    int64_t frame_seq;
    int64_t timestamp_ms;
    int     image_width;     
    int     image_height;    
    std::vector<Detection> dets;

    FrameDetections() : frame_seq(0), timestamp_ms(0), image_width(1280), image_height(720) {}
};

}  

#endif  
