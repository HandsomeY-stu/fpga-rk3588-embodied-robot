/*
 * target_selector.hpp
 * 目标选择器，按类别、可靠性与身份连续性选出控制目标。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef TARGET_SELECTOR_HPP
#define TARGET_SELECTOR_HPP

#include "vision_types.hpp"
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>

namespace vision {

struct SelectorConfig {
    float              conf_threshold;    
    std::set<int>      allowed_classes;   
    int                min_area;          
    int                max_area;          
    int                image_width;       
    int                image_height;      
    int                prev_cx, prev_cy;  
    bool               prev_valid;        

    SelectorConfig()
        : conf_threshold(0.40f), min_area(0), max_area(0),
          image_width(1280), image_height(720),
          prev_cx(-1), prev_cy(-1), prev_valid(false) {}
};

struct SelectResult {
    bool               ok;
    vision::Detection  det;              
    int                total;            
    int                rejected_conf;    
    int                rejected_class;   
    int                rejected_geom;    

    SelectResult() : ok(false), total(0), rejected_conf(0), rejected_class(0), rejected_geom(0) {}
};

inline bool geom_valid(const Detection& d, int W, int H) {
    if (d.w <= 0 || d.h <= 0) return false;
    if (d.x < 0 || d.y < 0) return false;
    if (d.x + d.w > W || d.y + d.h > H) return false;
    return true;
}

inline SelectResult select_target(const FrameDetections& frame, const SelectorConfig& cfg) {
    SelectResult r;
    r.total = (int)frame.dets.size();

    std::vector<Detection> pass;
    for (size_t i = 0; i < frame.dets.size(); ++i) {
        const Detection& d = frame.dets[i];

        if (d.confidence < cfg.conf_threshold) { r.rejected_conf++; continue; }

        if (!cfg.allowed_classes.empty() &&
            cfg.allowed_classes.count(d.class_id) == 0) { r.rejected_class++; continue; }

        if (!geom_valid(d, cfg.image_width, cfg.image_height)) { r.rejected_geom++; continue; }
        if (d.area() < cfg.min_area) { r.rejected_geom++; continue; }
        if (cfg.max_area > 0 && d.area() > cfg.max_area) { r.rejected_geom++; continue; }
        pass.push_back(d);
    }

    if (pass.empty()) { r.ok = false; return r; }

    std::sort(pass.begin(), pass.end(),
        [&](const Detection& a, const Detection& b) {
            if (a.confidence != b.confidence) return a.confidence > b.confidence;
            if (cfg.prev_valid) {
                double da = std::hypot(double(a.cx() - cfg.prev_cx), double(a.cy() - cfg.prev_cy));
                double db = std::hypot(double(b.cx() - cfg.prev_cx), double(b.cy() - cfg.prev_cy));
                if (da != db) return da < db;
            }
            return a.class_id < b.class_id;
        });

    r.ok = true;
    r.det = pass[0];
    return r;
}

}  

#endif  
