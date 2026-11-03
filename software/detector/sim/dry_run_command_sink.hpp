/* dry_run_command_sink.hpp：离线运动命令接收器，记录命令而不访问串口或电机。 */
#ifndef DRY_RUN_COMMAND_SINK_HPP
#define DRY_RUN_COMMAND_SINK_HPP

#include "uart_control.hpp"
#include <cstdint>
#include <vector>
#include <string>
#include <cstdio>

namespace sim {

// 一条已记录的命令（dry-run 不落地到硬件）
struct CommandRecord {
    int64_t             seq;            // 应用层命令序号（单调）
    uint8_t             fseq;           // 协议帧 SEQ
    uint8_t             cmd;            // 命令码
    uint8_t             dir;            // 方向（仅 CMD_MOTION 有意义；否则 0）
    uint8_t             speed;          // 速度（仅 CMD_MOTION 有意义）
    std::string         hexstr;         // 完整帧十六进制
    std::string         logic;          // 逻辑命令名（STOP/FORWARD/LEFT/RIGHT/...）
    int64_t             timestamp_ms;
    std::string         reject_reason;  // 空 = 正常发出；非空 = 被拒绝（不产生帧）
};

// 只记录、不发串口的 sink
class DryRunCommandSink {
public:
    // 提交一条运动命令。reject_reason 非空表示"被上层策略拒绝"，不记录帧。
    void emit_motion(uint8_t fseq, uint8_t dir, uint8_t speed,
                     const std::string& logic, int64_t ts_ms) {
        std::vector<uint8_t> f = uartctl::motion_frame(fseq, dir, speed);
        CommandRecord r;
        r.seq = (int64_t)records_.size();
        r.fseq = fseq;
        r.cmd = uartctl::CMD_MOTION;
        r.dir = dir; r.speed = speed;
        r.hexstr = uartctl::hex(f);
        r.logic = logic;
        r.timestamp_ms = ts_ms;
        r.reject_reason = "";
        records_.push_back(r);
    }

    void emit_stop(uint8_t fseq, const std::string& logic, int64_t ts_ms) {
        std::vector<uint8_t> f = uartctl::stop_frame(fseq);
        CommandRecord r;
        r.seq = (int64_t)records_.size();
        r.fseq = fseq;
        r.cmd = uartctl::CMD_STOP;
        r.dir = uartctl::DIR_STOP; r.speed = 0;
        r.hexstr = uartctl::hex(f);
        r.logic = logic;
        r.timestamp_ms = ts_ms;
        r.reject_reason = "";
        records_.push_back(r);
    }

    // 记录一次"被拒绝"（不产生帧）
    void emit_rejected(const std::string& logic, const std::string& reason, int64_t ts_ms) {
        CommandRecord r;
        r.seq = (int64_t)records_.size();
        r.fseq = 0;
        r.cmd = 0; r.dir = 0; r.speed = 0;
        r.hexstr = "";
        r.logic = logic;
        r.timestamp_ms = ts_ms;
        r.reject_reason = reason;
        records_.push_back(r);
    }

    const std::vector<CommandRecord>& records() const { return records_; }
    size_t count() const { return records_.size(); }
    size_t count_motion() const {
        size_t n = 0; for (size_t i = 0; i < records_.size(); ++i)
            if (records_[i].reject_reason.empty() && records_[i].cmd == uartctl::CMD_MOTION) ++n;
        return n;
    }
    size_t count_stop() const {
        size_t n = 0; for (size_t i = 0; i < records_.size(); ++i)
            if (records_[i].reject_reason.empty() && records_[i].cmd == uartctl::CMD_STOP) ++n;
        return n;
    }
    size_t count_rejected() const {
        size_t n = 0; for (size_t i = 0; i < records_.size(); ++i)
            if (!records_[i].reject_reason.empty()) ++n;
        return n;
    }

    void reset() { records_.clear(); }

private:
    std::vector<CommandRecord> records_;
};

} // namespace sim

#endif // DRY_RUN_COMMAND_SINK_HPP
