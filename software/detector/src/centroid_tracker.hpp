/*
 * centroid_tracker.hpp
 * 轻量质心目标跟踪，维护目标关联、速度估计与丢失期限。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef CENTROID_TRACKER_HPP
#define CENTROID_TRACKER_HPP

#include "vision_types.hpp"
#include <cmath>
#include <cstdio>

namespace vision {

struct TrackerConfig {
    int    confirm_frames;     
    int    lost_timeout_ms;    
    int    max_missed_frames;  
    double max_center_dist;    
    double min_iou;            

    TrackerConfig()
        : confirm_frames(3), lost_timeout_ms(1000), max_missed_frames(5),
          max_center_dist(160.0), min_iou(0.1) {}
};

struct TrackState {
    bool    active;           
    int     track_id;         
    bool    confirmed;        
    bool    lost;             
    bool    occluded;         
    int     consecutive_hits; 
    int     missed_frames;    
    int64_t last_hit_ms;      
    int64_t last_frame_seq;   
    Detection last_det;       
    double  vx, vy;           
    int     cx, cy;           

    TrackState() : active(false), track_id(0), confirmed(false), lost(false),
                   occluded(false), consecutive_hits(0), missed_frames(0),
                   last_hit_ms(0), last_frame_seq(-1), vx(0), vy(0), cx(0), cy(0) {}
};

inline double iou(const Detection& a, const Detection& b) {
    int ax1 = a.x, ay1 = a.y, ax2 = a.x + a.w, ay2 = a.y + a.h;
    int bx1 = b.x, by1 = b.y, bx2 = b.x + b.w, by2 = b.y + b.h;
    int ix1 = ax1 > bx1 ? ax1 : bx1;
    int iy1 = ay1 > by1 ? ay1 : by1;
    int ix2 = ax2 < bx2 ? ax2 : bx2;
    int iy2 = ay2 < by2 ? ay2 : by2;
    int iw = ix2 - ix1, ih = iy2 - iy1;
    if (iw <= 0 || ih <= 0) return 0.0;
    double inter = (double)iw * ih;
    double ua = (double)a.area() + (double)b.area() - inter;
    return ua > 0 ? inter / ua : 0.0;
}

class CentroidTracker {
public:
    explicit CentroidTracker(const TrackerConfig& cfg) : cfg_(cfg) {}

    TrackState update(const Detection* det, const FrameDetections& frame) {
        int64_t now = frame.timestamp_ms;

        if (now < st_.last_hit_ms && st_.active) {

            advance_miss(now);
            return st_;
        }

        if (det != NULL) {

            bool match = !st_.active || !st_.confirmed || within_gate(*det);
            if (match) {
                hit(*det, now, frame.frame_seq);
            } else {

                st_.track_id += 1;
                reset_to(*det, now, frame.frame_seq);
            }
        } else {
            advance_miss(now);
        }
        return st_;
    }

    const TrackState& state() const { return st_; }

    void print_config() const {
        printf("  [tracker] confirm_frames=%d lost_timeout_ms=%d max_missed_frames=%d "
               "max_center_dist=%.1f min_iou=%.2f\n",
               cfg_.confirm_frames, cfg_.lost_timeout_ms, cfg_.max_missed_frames,
               cfg_.max_center_dist, cfg_.min_iou);
    }

private:
    bool within_gate(const Detection& d) const {
        double dist = std::hypot(double(d.cx() - st_.cx), double(d.cy() - st_.cy));
        if (dist <= cfg_.max_center_dist) return true;

        return iou(d, st_.last_det) >= cfg_.min_iou;
    }

    void hit(const Detection& d, int64_t now, int64_t seq) {

        if (st_.active && st_.confirmed && now > st_.last_hit_ms) {
            double dt = double(now - st_.last_hit_ms);
            if (dt > 0) {
                st_.vx = double(d.cx() - st_.cx) / dt;
                st_.vy = double(d.cy() - st_.cy) / dt;
            }
        }
        st_.active = true;
        st_.lost = false;
        st_.occluded = false;
        st_.missed_frames = 0;
        st_.consecutive_hits += 1;
        if (st_.consecutive_hits >= cfg_.confirm_frames) st_.confirmed = true;
        st_.last_hit_ms = now;
        st_.last_frame_seq = seq;
        st_.last_det = d;
        st_.cx = d.cx();
        st_.cy = d.cy();
    }

    void reset_to(const Detection& d, int64_t now, int64_t seq) {
        st_ = TrackState();
        st_.active = true;
        st_.consecutive_hits = 1;
        st_.last_hit_ms = now;
        st_.last_frame_seq = seq;
        st_.last_det = d;
        st_.cx = d.cx();
        st_.cy = d.cy();
        if (st_.consecutive_hits >= cfg_.confirm_frames) st_.confirmed = true;
    }

    void advance_miss(int64_t now) {
        if (!st_.active) return;                  
        st_.missed_frames += 1;

        bool time_exceeded = (st_.last_hit_ms > 0) && (now - st_.last_hit_ms) >= cfg_.lost_timeout_ms;
        if (st_.missed_frames <= cfg_.max_missed_frames && !time_exceeded) {
            st_.occluded = true;                  
        } else {

            st_.occluded = false;
            st_.lost = true;
            st_.confirmed = false;                
            st_.consecutive_hits = 0;             
        }
    }

    TrackerConfig cfg_;
    TrackState    st_;
};

}  

#endif  
