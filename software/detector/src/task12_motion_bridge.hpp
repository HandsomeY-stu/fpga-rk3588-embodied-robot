/*
 * task12_motion_bridge.hpp
 * 视觉观测到运动控制接口的桥接层，管理目标有效性与命令转换。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef TASK12_MOTION_BRIDGE_HPP
#define TASK12_MOTION_BRIDGE_HPP

#include "vision_types.hpp"
#include "centroid_tracker.hpp"
#include "uart_control.hpp"
#include "../sim/dry_run_command_sink.hpp"
#include <cstdio>
#include <string>

namespace task12 {

enum class CtrlState {
    IDLE,
    TRACK_CONFIRM,
    FOLLOW_LEFT,
    FOLLOW_RIGHT,
    FOLLOW_FORWARD,
    NEAR_STOP,
    LOST_STOP,
    FAULT_STOP
};

inline const char* state_name(CtrlState s) {
    switch (s) {
        case CtrlState::IDLE:           return "IDLE";
        case CtrlState::TRACK_CONFIRM:  return "TRACK_CONFIRM";
        case CtrlState::FOLLOW_LEFT:    return "FOLLOW_LEFT";
        case CtrlState::FOLLOW_RIGHT:   return "FOLLOW_RIGHT";
        case CtrlState::FOLLOW_FORWARD: return "FOLLOW_FORWARD";
        case CtrlState::NEAR_STOP:      return "NEAR_STOP";
        case CtrlState::LOST_STOP:      return "LOST_STOP";
        case CtrlState::FAULT_STOP:     return "FAULT_STOP";
    }
    return "?";
}

struct BridgeConfig {
    int     dead_zone;           
    int     near_area;           
    int     max_speed;           
    int     turn_speed;          
    int64_t min_cmd_interval_ms; 
    int64_t stale_data_ms;       
    int     image_width;         

    BridgeConfig()
        : dead_zone(40), near_area(60000), max_speed(60), turn_speed(35),
          min_cmd_interval_ms(200), stale_data_ms(1000), image_width(1280) {}
};

struct BridgeDecision {
    CtrlState  state;
    bool       emitted;          
    uint8_t    dir;              
    uint8_t    speed;
    std::string logic;           
    std::string reason;          
    int        err_px;           

    BridgeDecision() : state(CtrlState::IDLE), emitted(false),
                       dir(uartctl::DIR_STOP), speed(0), err_px(0) {}
};

class Task12MotionBridge {
public:
    Task12MotionBridge(const BridgeConfig& cfg, const vision::TrackerConfig& tcfg)
        : cfg_(cfg), tracker_(tcfg), fseq_(0) {}

    BridgeDecision step(const vision::FrameDetections& frame, sim::DryRunCommandSink& sink);

    void force_stop(sim::DryRunCommandSink& sink, int64_t ts_ms, bool fault);

    const BridgeConfig& cfg() const { return cfg_; }
    const vision::CentroidTracker& tracker() const { return tracker_; }
    CtrlState state() const { return state_; }

private:
    void do_stop(sim::DryRunCommandSink& sink, int64_t ts_ms, const std::string& logic,
                 CtrlState next);

    BridgeConfig           cfg_;
    vision::CentroidTracker tracker_;
    CtrlState              state_ = CtrlState::IDLE;
    uint8_t                fseq_ = 0;
    int64_t                last_emit_ms_ = 0;
};

inline BridgeDecision Task12MotionBridge::step(
        const vision::FrameDetections& frame, sim::DryRunCommandSink& sink) {
    BridgeDecision d;
    int64_t now = frame.timestamp_ms;
    int cx_img = cfg_.image_width / 2;    

    vision::SelectorConfig sc;
    sc.conf_threshold = 0.40f;             
    sc.image_width = cfg_.image_width;
    sc.image_height = 720;
    const vision::TrackState& ts = tracker_.state();
    if (ts.active && ts.confirmed) { sc.prev_cx = ts.cx; sc.prev_cy = ts.cy; sc.prev_valid = true; }

    vision::SelectResult sel = vision::select_target(frame, sc);

    const vision::Detection* detp = sel.ok ? &sel.det : NULL;
    vision::TrackState st = tracker_.update(detp, frame);

    if (now < 0) {    
        do_stop(sink, now, "FAULT_STOP(bad_ts)", CtrlState::FAULT_STOP);
        d.state = CtrlState::FAULT_STOP; d.emitted = true; d.dir = uartctl::DIR_STOP;
        d.logic = "FAULT_STOP"; d.reason = "bad_timestamp";
        return d;
    }

    if (!st.active || st.lost) {
        do_stop(sink, now, "LOST_STOP", CtrlState::LOST_STOP);
        d.state = CtrlState::LOST_STOP; d.emitted = true; d.dir = uartctl::DIR_STOP;
        d.logic = "LOST_STOP"; d.reason = "lost";
        return d;
    }

    if (now - st.last_hit_ms > cfg_.stale_data_ms) {
        do_stop(sink, now, "LOST_STOP(stale)", CtrlState::LOST_STOP);
        d.state = CtrlState::LOST_STOP; d.emitted = true; d.dir = uartctl::DIR_STOP;
        d.logic = "LOST_STOP"; d.reason = "stale";
        return d;
    }

    if (!st.confirmed) {

        d.state = CtrlState::TRACK_CONFIRM; d.emitted = false;
        d.dir = uartctl::DIR_STOP; d.logic = "TRACK_CONFIRM"; d.reason = "not_confirmed";
        state_ = CtrlState::TRACK_CONFIRM;
        return d;
    }

    int err = st.cx - cx_img;
    d.err_px = err;
    d.state = CtrlState::FOLLOW_FORWARD;

    uint8_t dir = uartctl::DIR_FWD;
    uint8_t speed = (uint8_t)cfg_.max_speed;
    std::string logic = "FORWARD";

    if (st.last_det.area() >= cfg_.near_area) {

        do_stop(sink, now, "NEAR_STOP", CtrlState::NEAR_STOP);
        d.state = CtrlState::NEAR_STOP; d.emitted = true; d.dir = uartctl::DIR_STOP;
        d.logic = "NEAR_STOP"; d.reason = "near";
        return d;
    }

    if (err < -cfg_.dead_zone) { dir = uartctl::DIR_LEFT;  speed = (uint8_t)cfg_.turn_speed; logic = "LEFT";  d.state = CtrlState::FOLLOW_LEFT; }
    else if (err > cfg_.dead_zone) { dir = uartctl::DIR_RIGHT; speed = (uint8_t)cfg_.turn_speed; logic = "RIGHT"; d.state = CtrlState::FOLLOW_RIGHT; }

    bool debounced = (now - last_emit_ms_) >= cfg_.min_cmd_interval_ms;
    if (!debounced) {
        d.emitted = false; d.dir = dir; d.speed = speed;
        d.logic = logic; d.reason = "debounced";
        state_ = d.state;
        return d;
    }

    sink.emit_motion(fseq_++, dir, speed, logic, now);
    last_emit_ms_ = now;
    state_ = d.state;
    d.emitted = true; d.dir = dir; d.speed = speed; d.logic = logic; d.reason = "";
    return d;
}

inline void Task12MotionBridge::do_stop(sim::DryRunCommandSink& sink, int64_t ts_ms,
                                         const std::string& logic, CtrlState next) {
    sink.emit_stop(fseq_++, logic, ts_ms);
    last_emit_ms_ = ts_ms;
    state_ = next;
}

inline void Task12MotionBridge::force_stop(sim::DryRunCommandSink& sink, int64_t ts_ms, bool fault) {
    do_stop(sink, ts_ms, fault ? "FAULT_STOP" : "ESTOP_STOP",
            fault ? CtrlState::FAULT_STOP : CtrlState::LOST_STOP);
}

}  

#endif  
