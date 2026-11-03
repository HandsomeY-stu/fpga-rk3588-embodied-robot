/*
 * realtime_ssd_jsonl.cpp
 * 检测JSONL输出实现，保留逐帧观测的离线接口。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#include "rknn_api.h"
#include "v4l2_capture.hpp"
#include "ssd_postprocess.hpp"

#include "../uart/uart_vision.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <csignal>
#include <unistd.h>       

static double now_sec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

static long read_long_file(const char *path) {
    std::ifstream f(path);
    long v = -1;
    if (f.is_open()) f >> v;
    return v;
}

static std::string read_str_file(const char *path) __attribute__((unused));
static std::string read_str_file(const char *path) {
    std::ifstream f(path);
    std::string s;
    if (f.is_open()) std::getline(f, s);
    return s;
}

static long read_npu_freq() {
    const char *p = "/sys/class/devfreq/fdab0000.npu/cur_freq";
    long v = read_long_file(p);
    if (v >= 0) return v;
    return read_long_file("/sys/kernel/debug/clk/clk_summary");   
}

static long read_temp_mC() {
    const char *cands[] = {
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/thermal/thermal_zone1/temp",
    };
    for (auto p : cands) {
        long v = read_long_file(p);
        if (v > 0) return v;
    }
    return -1;
}

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

static bool load_bmp_bgr24(const char *path, std::vector<uint8_t> &out,
                           int &w, int &h) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    uint8_t hdr[54];
    if (fread(hdr, 1, 54, f) != 54) { fclose(f); return false; }
    if (hdr[0] != 'B' || hdr[1] != 'M') { fclose(f); return false; }
    uint32_t data_off;
    memcpy(&data_off, hdr + 10, 4);
    int32_t wi, hi;
    memcpy(&wi, hdr + 18, 4);
    memcpy(&hi, hdr + 22, 4);
    uint16_t bpp;
    memcpy(&bpp, hdr + 28, 2);
    if (bpp != 24) { fclose(f); return false; }
    bool bottom_up = hi > 0;
    w = wi;
    h = (hi < 0) ? -hi : hi;
    int row_raw = w * 3;
    int row_pad = (4 - (row_raw % 4)) % 4;
    int row_tot = row_raw + row_pad;
    std::vector<uint8_t> raw((size_t)row_tot * h);
    fseek(f, data_off, SEEK_SET);
    if (fread(raw.data(), 1, raw.size(), f) != raw.size()) { fclose(f); return false; }
    fclose(f);

    out.assign((size_t)w * h * 3, 0);
    for (int y = 0; y < h; y++) {
        int src_y = bottom_up ? (h - 1 - y) : y;
        memcpy(out.data() + (size_t)y * row_raw,
               raw.data() + (size_t)src_y * row_tot, row_raw);
    }
    return true;
}

static int run_image_mode(const std::string &img_path,
                          const std::string &model_path,
                          const std::string &priors_path,
                          const std::string &labels_path,
                          float conf_t, float nms_t,
                          const std::string &dump_path, bool verbose) {
    std::vector<uint8_t> model;
    {
        std::ifstream mf(model_path, std::ios::binary | std::ios::ate);
        if (!mf.is_open()) { fprintf(stderr, "[err] 打不开模型\n"); return 3; }
        size_t n = (size_t)mf.tellg(); mf.seekg(0);
        model.resize(n); mf.read((char *)model.data(), n);
    }
    rknn_context ctx = 0;
    if (rknn_init(&ctx, model.data(), (uint32_t)model.size(), 0, NULL) != RKNN_SUCC) {
        fprintf(stderr, "[err] rknn_init 失败\n"); return 4;
    }
    printf("[img] rknn_init OK\n");

    SSDPostProcess post;
    if (!post.loadAnchors(priors_path, labels_path)) return 6;

    std::vector<uint8_t> bgr;
    int W = 0, H = 0;
    if (!load_bmp_bgr24(img_path.c_str(), bgr, W, H)) {
        fprintf(stderr, "[err] 读 BMP 失败: %s（需 24bit 未压缩 BMP）\n", img_path.c_str());
        rknn_destroy(ctx); return 7;
    }
    printf("[img] 载入 %s : %dx%d\n", img_path.c_str(), W, H);

    std::vector<uint8_t> in_buf(300 * 300 * 3);
    preprocess_bgr24_to_300(bgr.data(), W, H, W * 3, in_buf.data());

    rknn_input in;
    memset(&in, 0, sizeof(in));
    in.index = 0; in.buf = in_buf.data();
    in.size = (uint32_t)in_buf.size();
    in.type = RKNN_TENSOR_UINT8; in.fmt = RKNN_TENSOR_NHWC; in.pass_through = 0;
    if (rknn_inputs_set(ctx, 1, &in) != RKNN_SUCC) { fprintf(stderr, "[err] inputs_set\n"); return 8; }
    if (rknn_run(ctx, NULL) != RKNN_SUCC) { fprintf(stderr, "[err] run\n"); return 8; }

    rknn_output outs[2];
    memset(outs, 0, sizeof(outs));

    outs[0].index = 0; outs[1].index = 1;
    for (auto &o : outs) { o.want_float = 1; o.is_prealloc = 0; }
    if (rknn_outputs_get(ctx, 2, outs, NULL) != RKNN_SUCC) { fprintf(stderr, "[err] outputs_get\n"); return 8; }

    std::vector<Detection> dets = post.decode((const float *)outs[0].buf,
                                             (const float *)outs[1].buf,
                                             W, H, conf_t, nms_t);
    printf("[img] 检出 %zu 个目标:\n", dets.size());
    for (auto &d : dets) {
        const std::string &nm = post.labelOf(d.cls);
        printf("   [%2d] %-16s %.3f  center=(%.0f,%.0f) wh=(%.0f,%.0f)\n",
               d.cls, nm.empty() ? "?" : nm.c_str(), d.conf, d.px, d.py, d.pw, d.ph);
    }
    if (!dump_path.empty()) {
        draw_boxes_bgr(bgr.data(), W, H, W * 3, dets);
        if (save_bmp_bgr24(dump_path.c_str(), bgr.data(), W, H, W * 3))
            printf("[img] 已存带框图: %s\n", dump_path.c_str());
    }
    rknn_outputs_release(ctx, 2, outs);
    rknn_destroy(ctx);
    return 0;
}

int main(int argc, char **argv) {
    std::string model_path, priors_path, labels_path, dev = "/dev/video0";
    std::string dump_path, image_path;
    std::string uart_dev = "/dev/ttyS0";    
    bool   uart_en = true;                  
    int    secs   = 600;          
    float  conf_t = 0.40f;
    float  nms_t  = 0.45f;

    float  cg_r = 1.0f, cg_g = 1.0f, cg_b = 1.0f;
    bool   verbose = false;
    int    warmup  = 5;           

    bool   dry_run_control = true;    
    std::string target_label;         
    int    confirm_frames = 3;        
    int    lost_timeout_ms = 1000;    

    std::string   jsonl_path;
    unsigned long jsonl_seq = 0;
    FILE*         jsonl_fp  = nullptr;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&](void) -> const char * {
            if (i + 1 >= argc) { fprintf(stderr, "[err] %s 缺参数\n", a.c_str()); exit(2); }
            return argv[++i];
        };
        if      (a == "-m" || a == "--model")  model_path  = next();
        else if (a == "-a" || a == "--priors") priors_path = next();
        else if (a == "-l" || a == "--labels") labels_path = next();
        else if (a == "-d" || a == "--dev")    dev         = next();
        else if (a == "--secs")   secs   = atoi(next());
        else if (a == "--conf")   conf_t = atof(next());
        else if (a == "--nms")    nms_t  = atof(next());
        else if (a == "--dump")   dump_path = next();
        else if (a == "--uart-dev") uart_dev = next();
        else if (a == "--no-uart")  uart_en  = false;
        else if (a == "--image")  image_path = next();
        else if (a == "--warmup") warmup = atoi(next());

        else if (a == "--dry-run-control") dry_run_control = true;
        else if (a == "--no-dry-run-control") dry_run_control = false;
        else if (a == "--target-label") target_label = next();
        else if (a == "--confirm-frames") confirm_frames = atoi(next());
        else if (a == "--lost-timeout-ms") lost_timeout_ms = atoi(next());
        else if (a == "--jsonl-out") jsonl_path = next();    
        else if (a == "--color-gain") {

            const char *s = next();
            if (sscanf(s, "%f,%f,%f", &cg_r, &cg_g, &cg_b) != 3) {
                fprintf(stderr, "[err] --color-gain 需要 \"R,G,B\" 三个数\n");
                return 2;
            }
        }
        else if (a == "--verbose") verbose = true;
        else if (a == "-h" || a == "--help") {
            printf("用法: %s -m model.rknn -a box_priors.txt [-l labels.txt] "
                   "[-d /dev/video0] [--secs 600] [--conf 0.4] [--nms 0.45] "
                   "[--dump out.bmp] [--verbose] "
                   "[--uart-dev /dev/ttyS0] [--no-uart] "
                   "[--color-gain R,G,B] "
                   "[--dry-run-control|--no-dry-run-control] [--target-label NAME] "
                   "[--confirm-frames N] [--lost-timeout-ms MS] [--jsonl-out FILE]\n", argv[0]);
            return 0;
        }
        else { fprintf(stderr, "[err] 未知参数 %s\n", a.c_str()); return 2; }
    }
    if (model_path.empty() || priors_path.empty()) {
        fprintf(stderr, "必须指定 -m <model.rknn> 与 -a <box_priors.txt>\n");
        return 2;
    }

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    set_color_gain(cg_r, cg_g, cg_b);
    if (!(cg_r == 1.0f && cg_g == 1.0f && cg_b == 1.0f))
        printf("[color] ★方案A 偏绿校正已启用: R=%.3f G=%.3f B=%.3f\n",
               cg_r, cg_g, cg_b);

    if (!image_path.empty()) {
        return run_image_mode(image_path, model_path, priors_path, labels_path,
                              conf_t, nms_t, dump_path, verbose);
    }

    printf("========================================\n");
    printf(" RK3588 实时 SSD 检测 (HDMI-IN 常驻循环)\n");
    printf("========================================\n");
    printf(" 模型  : %s\n", model_path.c_str());
    printf(" priors: %s\n", priors_path.c_str());
    printf(" 设备  : %s\n", dev.c_str());
    printf(" 阈值  : conf=%.2f nms=%.2f\n", conf_t, nms_t);
    printf(" 时长  : %d 秒 (warmup %d 帧)\n", secs, warmup);
    printf("========================================\n");
    fflush(stdout);

    std::ifstream mf(model_path, std::ios::binary | std::ios::ate);
    if (!mf.is_open()) { fprintf(stderr, "[err] 打不开模型 %s\n", model_path.c_str()); return 3; }
    size_t msize = (size_t)mf.tellg();
    mf.seekg(0);
    std::vector<uint8_t> model(msize);
    mf.read((char *)model.data(), msize);
    mf.close();
    printf("[main] 模型已载入 %zu 字节 (%.2f MB)\n", msize, msize / 1048576.0);

    rknn_context ctx = 0;
    double t0 = now_sec();
    int ret = rknn_init(&ctx, model.data(), (uint32_t)msize, 0, NULL);
    double t_init = now_sec() - t0;
    if (ret != RKNN_SUCC) {
        fprintf(stderr, "[err] rknn_init 失败 ret=%d\n", ret);
        return 4;
    }
    printf("[main] rknn_init 成功，耗时 %.3f s（此后循环内不再重载）\n", t_init);

    rknn_input_output_num ion;
    memset(&ion, 0, sizeof(ion));
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &ion, sizeof(ion));
    printf("[main] query(IN_OUT_NUM) ret=%d size=%zu\n", ret, sizeof(ion));
    fflush(stdout);
    if (ret != RKNN_SUCC) {
        fprintf(stderr, "[err] 查询输入输出数量失败\n"); return 5;
    }
    printf("[main] 输入 %u 个，输出 %u 个\n", ion.n_input, ion.n_output);
    fflush(stdout);
    if (ion.n_input < 1 || ion.n_output < 2) {
        fprintf(stderr, "[err] 本程序假定 SSD 模型为 1 输入 2 输出(loc,conf)\n");
        return 5;
    }

    rknn_tensor_attr ia;
    memset(&ia, 0, sizeof(ia));
    ia.index = 0;
    ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &ia, sizeof(ia));
    printf("[main] query(INPUT_ATTR) ret=%d attr_size=%zu\n", ret, sizeof(ia));
    fflush(stdout);
    int in_w = 300, in_h = 300, in_c = 3;
    if (ia.n_dims >= 4 && ia.fmt == RKNN_TENSOR_NHWC) {
        in_h = ia.dims[1]; in_w = ia.dims[2]; in_c = ia.dims[3];
    } else if (ia.n_dims >= 4) {
        in_c = ia.dims[1]; in_h = ia.dims[2]; in_w = ia.dims[3];
    }
    printf("[main] 输入张量: %dx%dx%d type=%d fmt=%d\n",
           in_w, in_h, in_c, (int)ia.type, (int)ia.fmt);
    if (in_w != 300 || in_h != 300) {
        fprintf(stderr, "[warn] 输入不是 300x300 —— 本项目预处理按 300x300 写死，"
                        "如换模型需同步改 preprocess 与 anchors 数量\n");
    }

    const size_t IN_ELEMS = (size_t)in_w * in_h * in_c;

    std::vector<uint8_t> in_buf(IN_ELEMS);

    SSDPostProcess post;
    if (!post.loadAnchors(priors_path, labels_path)) {
        fprintf(stderr, "[err] 载入 anchors 失败\n"); return 6;
    }
    if (post.anchorCount() != 1917) {
        printf("[warn] anchors=%zu，模型标称 1917；请确认 box_priors 与模型配套\n",
               post.anchorCount());
    }

    V4L2Capture cap;
    if (!cap.open(dev, 1280, 720, 4)) {
        fprintf(stderr, "[err] 打开 %s 失败: %s\n", dev.c_str(), cap.lastError().c_str());
        fprintf(stderr, "       HDMI-IN 需先锁定：请确认 FPGA HDMI-OUT 已连接并输出\n");
        rknn_destroy(ctx);
        return 7;
    }
    int W = cap.width(), H = cap.height(), BPL = (int)cap.bytesPerLine();
    printf("[main] 采集: %dx%d bpl=%d frame=%zu\n", W, H, BPL, cap.frameBytes());

    if (uart_en) {
        if (!uartvis::vis_uart_init(uart_dev.c_str(), 115200))
            printf("[main] [warn] UART 未就绪，视觉帧发送禁用（采集/推理继续）\n");
    } else {
        printf("[main] UART 发送被 --no-uart 显式关闭\n");
    }

    std::vector<float> out_loc, out_conf;
    std::vector<rknn_output> outs(2);
    bool dump_done = false;

    double loop_start = now_sec();
    double last_report = loop_start;
    long   frames = 0, drops = 0;
    long   inf_frames = 0;
    double cap_fps = 0, inf_fps = 0, e2e_ms = 0;
    int    last_nbox = 0;
    double sum_inf = 0;

    int    consec_fail = 0;                  
    long   capture_recoveries = 0;           
    long   capture_recovery_failures = 0;    
    std::string last_capture_error;          
    double last_ok_frame_t = loop_start;     
    int    exit_code = 0;                    
    int    reopen_streak = 0;                
    bool   unrecoverable = false;            
    const int RECOVERY_THRESHOLD    = 3;     
    const int RECOVERY_MAX_ATTEMPTS = 3;     
    const int RECOVERY_BACKOFF_MS[3] = {100, 250, 500};
    const int MAX_REOPEN_STREAK     = 5;     

    printf("\n---- 进入实时循环（Ctrl-C 结束） ----\n");
    fflush(stdout);

    while (!g_stop) {
        if (now_sec() - loop_start > secs) break;

        uint8_t *raw = nullptr;
        size_t   rawbytes = 0;

        double tc0 = now_sec();
        if (!cap.grabFrame(&raw, &rawbytes, 2000)) {

            drops++;
            consec_fail++;
            last_capture_error = cap.lastError();

            const bool transient =
                (last_capture_error.find("EAGAIN")  != std::string::npos ||
                 last_capture_error.find("timeout") != std::string::npos ||
                 last_capture_error.find("select")  != std::string::npos);

            if (verbose)
                printf("[warn] 抓帧失败 #%d（%s）: %s\n", consec_fail,
                       transient ? "瞬时/无帧" : "持续", last_capture_error.c_str());

            if (consec_fail < RECOVERY_THRESHOLD) {
                usleep(2000);             
                continue;
            }

            cap.releaseFrame();
            bool recovered = false;
            for (int attempt = 0; attempt < RECOVERY_MAX_ATTEMPTS && !g_stop && !unrecoverable; attempt++) {
                int backoff = RECOVERY_BACKOFF_MS[attempt < 3 ? attempt : 2];
                printf("[recover] 连续抓帧失败 %d 次（最后错误: %s）；"
                       "第 %d/%d 次重开 %s，退避 %d ms\n",
                       consec_fail, last_capture_error.c_str(),
                       attempt + 1, RECOVERY_MAX_ATTEMPTS,
                       cap.device().c_str(), backoff);
                fflush(stdout);
                usleep(backoff * 1000);

                if (cap.reopen()) {
                    recovered = true;
                    capture_recoveries++;

                    if (cap.width() != W || cap.height() != H ||
                        (int)cap.bytesPerLine() != BPL) {
                        printf("[recover] ⚠ 帧几何变化: %dx%d bpl=%d → %dx%d bpl=%u，已同步\n",
                               W, H, BPL, cap.width(), cap.height(), cap.bytesPerLine());
                        W   = cap.width();
                        H   = cap.height();
                        BPL = (int)cap.bytesPerLine();
                    }
                    printf("[recover] ✓ 设备重开成功（累计恢复 %ld 次），预热 %d 帧\n",
                           capture_recoveries, warmup);
                    fflush(stdout);

                    if (++reopen_streak > MAX_REOPEN_STREAK) {
                        fprintf(stderr,
                                "[fatal] 自上次成功帧以来已连续重开 %d 次仍无稳定帧，"
                                "判定不可自愈，退出（设备 %s，最后错误 %s）\n",
                                reopen_streak, cap.device().c_str(),
                                last_capture_error.c_str());
                        exit_code = 21;
                        unrecoverable = true;
                        break;
                    }

                    for (int k = 0; k < warmup && !g_stop; k++) {
                        uint8_t *d2 = nullptr; size_t n2 = 0;
                        if (cap.grabFrame(&d2, &n2, 2000)) cap.releaseFrame();
                        else usleep(5000);
                    }
                    break;
                }
                capture_recovery_failures++;
                printf("[recover] ✗ 第 %d/%d 次重开失败: %s\n",
                       attempt + 1, RECOVERY_MAX_ATTEMPTS, cap.lastError().c_str());
                fflush(stdout);
            }

            if (!recovered) {
                if (g_stop) {

                    printf("[recover] 收到停止信号，中止恢复尝试（按正常收尾，不计失败）\n");
                    break;
                }

                if (uartvis::vis_uart_ready()) {
                    const uartvis::Stats &us0 = uartvis::stats();
                    printf("[recover] 恢复失败时 UART 累计: tx_ok=%lu echo_ok=%lu "
                           "timeout=%lu pending=%zu\n",
                           us0.tx_ok, us0.echo_ok, us0.echo_timeout,
                           uartvis::pending_count());
                }
                fprintf(stderr,
                        "[fatal] 抓帧自愈失败：连续失败 %d 次，重开尝试 %d 次均失败；"
                        "最后错误=%s（设备 %s）\n",
                        consec_fail, RECOVERY_MAX_ATTEMPTS,
                        cap.lastError().c_str(), cap.device().c_str());
                exit_code = 20;
                break;
            }
            if (unrecoverable) break;    
            consec_fail = 0;
            last_ok_frame_t = now_sec();
            continue;
        }

        consec_fail = 0;
        reopen_streak = 0;
        last_ok_frame_t = now_sec();
        double tc1 = now_sec(); (void)tc1;
        frames++;

        if (frames <= warmup) {

            preprocess_bgr24_to_300(raw, W, H, BPL, in_buf.data());
            rknn_input in;
            memset(&in, 0, sizeof(in));
            in.index = 0; in.buf = in_buf.data();
            in.size = (uint32_t)IN_ELEMS;
            in.type = RKNN_TENSOR_UINT8; in.fmt = RKNN_TENSOR_NHWC;
            in.pass_through = 0;
            rknn_inputs_set(ctx, 1, &in);
            rknn_run(ctx, NULL);
            cap.releaseFrame();
            if (frames == warmup) printf("[main] 预热 %d 帧完成，开始统计\n", warmup);
            continue;
        }

        preprocess_bgr24_to_300(raw, W, H, BPL, in_buf.data());

        double ti0 = now_sec();

        rknn_input in;
        memset(&in, 0, sizeof(in));
        in.index = 0;
        in.buf   = in_buf.data();
        in.size  = (uint32_t)IN_ELEMS;
        in.type  = RKNN_TENSOR_UINT8;
        in.fmt   = RKNN_TENSOR_NHWC;
        in.pass_through = 0;

        ret = rknn_inputs_set(ctx, 1, &in);
        if (ret != RKNN_SUCC) { fprintf(stderr, "[err] inputs_set ret=%d\n", ret); break; }

        ret = rknn_run(ctx, NULL);
        if (ret != RKNN_SUCC) { fprintf(stderr, "[err] run ret=%d\n", ret); break; }

        memset(outs.data(), 0, sizeof(rknn_output) * 2);

        outs[0].index = 0; outs[1].index = 1;
        for (auto &o : outs) { o.want_float = 1; o.is_prealloc = 0; }
        ret = rknn_outputs_get(ctx, 2, outs.data(), NULL);
        if (ret != RKNN_SUCC) { fprintf(stderr, "[err] outputs_get ret=%d\n", ret); break; }

        double ti1 = now_sec();
        inf_frames++;

        const float *loc  = (const float *)outs[0].buf;    
        const float *conf = (const float *)outs[1].buf;    

        std::vector<Detection> dets = post.decode(loc, conf, W, H, conf_t, nms_t);
        last_nbox = (int)dets.size();

        if (uartvis::vis_uart_ready()) {
            if (!dets.empty()) {
                const Detection* best = &dets[0];
                for (size_t i = 1; i < dets.size(); i++)
                    if (dets[i].conf > best->conf) best = &dets[i];
                int cx = (int)(best->px + 0.5f), cy = (int)(best->py + 0.5f);
                int bw = (int)(best->pw + 0.5f), bh = (int)(best->ph + 0.5f);
                int cf = (int)(best->conf * 100.0f + 0.5f);    
                uartvis::vis_send_box(best->cls, cx, cy, bw, bh, cf);
            } else {
                uartvis::vis_send_invalid();
            }

            if (!jsonl_path.empty() && !jsonl_fp)
                jsonl_fp = fopen(jsonl_path.c_str(), "w");
            if (jsonl_fp) {
                if (!dets.empty()) {
                    const Detection* b2 = &dets[0];
                    for (size_t i = 1; i < dets.size(); i++)
                        if (dets[i].conf > b2->conf) b2 = &dets[i];
                    fprintf(jsonl_fp,
                            "{\"seq\":%lu,\"valid\":1,\"cls\":%d,\"cx\":%d,\"cy\":%d,"
                            "\"w\":%d,\"h\":%d,\"conf\":%d,\"n_dets\":%zu}\n",
                            (unsigned long)(jsonl_seq & 0xFFUL), b2->cls,
                            (int)(b2->px + 0.5f), (int)(b2->py + 0.5f),
                            (int)(b2->pw + 0.5f), (int)(b2->ph + 0.5f),
                            (int)(b2->conf * 100.0f + 0.5f), dets.size());
                } else {
                    fprintf(jsonl_fp,
                            "{\"seq\":%lu,\"valid\":0,\"cls\":0,\"cx\":0,\"cy\":0,"
                            "\"w\":0,\"h\":0,\"conf\":0,\"n_dets\":0}\n",
                            (unsigned long)(jsonl_seq & 0xFFUL));
                }
                fflush(jsonl_fp);
                jsonl_seq++;
            }
        }

        if (verbose) {
            for (auto &d : dets) {
                const std::string &nm = post.labelOf(d.cls);
                printf("   [%2d] %-16s %.3f  center=(%.0f,%.0f) wh=(%.0f,%.0f)\n",
                       d.cls, nm.empty() ? "?" : nm.c_str(), d.conf, d.px, d.py, d.pw, d.ph);
            }
        }

        if (!dump_done && !dump_path.empty() && !dets.empty()) {
            draw_boxes_bgr(raw, W, H, BPL, dets);
            if (save_bmp_bgr24(dump_path.c_str(), raw, W, H, BPL))
                printf("[main] 已保存带框图: %s (%zu 框)\n", dump_path.c_str(), dets.size());
            dump_done = true;
        }

        rknn_outputs_release(ctx, 2, outs.data());
        cap.releaseFrame();

        double tc2 = now_sec();
        e2e_ms = (tc2 - tc0) * 1000.0;
        sum_inf += (ti1 - ti0);

        double el = now_sec() - last_report;
        if (el >= 1.0) {
            cap_fps = frames / (now_sec() - loop_start + 1e-9);
            inf_fps = inf_frames / (now_sec() - loop_start + 1e-9);
            double avg_inf_ms = inf_frames ? (sum_inf / inf_frames) * 1000.0 : 0;

            long temp_mC = read_temp_mC();
            long npu_hz  = read_npu_freq();

            double sig_gap = now_sec() - last_ok_frame_t;    
            printf("[%6.1fs] 采集%.1f fps | 推理%.1f fps | 单次 %.1f ms | E2E %.1f ms "
                   "| 框%2d | 丢帧%ld | 恢复%ld/%ld | 无帧%.1fs | 温度 %.1f°C | NPU %s\n",
                   now_sec() - loop_start,
                   cap_fps, inf_fps, avg_inf_ms, e2e_ms,
                   last_nbox, drops,
                   capture_recoveries, capture_recovery_failures, sig_gap,
                   temp_mC > 0 ? temp_mC / 1000.0 : -1.0,
                   npu_hz > 0 ? (std::to_string(npu_hz / 1000000) + "MHz").c_str() : "n/a");
            fflush(stdout);
            last_report = now_sec();

            if (uartvis::vis_uart_ready()) {
                const uartvis::Stats &us = uartvis::stats();
                char age[64];
                uartvis::last_echo_age_str(age, (int)sizeof(age));
                printf("           UART tx_ok=%lu tx_err=%lu tx_short=%lu | "
                       "valid=%lu invalid=%lu | echo_ok=%lu mismatch=%lu crc_err=%lu timeout=%lu pending=%zu | 最近匹配=%s\n",
                       us.tx_ok, us.tx_err, us.tx_short,
                       us.valid_sent, us.invalid_sent,
                       us.echo_ok, us.echo_mismatch, us.echo_crc_err, us.echo_timeout,
                       uartvis::pending_count(), age);
                fflush(stdout);
            }

            if (dets.empty() && verbose) printf("   (本秒无框)\n");
        }
    }

    double total = now_sec() - loop_start;
    printf("\n---- 结束 ----\n");
    printf(" 总时长      : %.1f s\n", total);
    printf(" 采集帧      : %ld\n", frames);
    printf(" 推理帧      : %ld\n", inf_frames);
    printf(" 丢帧        : %ld\n", drops);

    printf(" 抓帧恢复    : 成功 %ld 次 / 失败 %ld 次（触发阈值 %d 连续失败，单次最多试 %d 次）\n",
           capture_recoveries, capture_recovery_failures,
           RECOVERY_THRESHOLD, RECOVERY_MAX_ATTEMPTS);
    if (!last_capture_error.empty())
        printf(" 最近抓帧错误: %s\n", last_capture_error.c_str());
    printf(" 平均采集FPS : %.2f\n", total > 0 ? frames / total : 0);
    printf(" 平均推理FPS : %.2f\n", total > 0 ? inf_frames / total : 0);
    printf(" 平均单次推理: %.2f ms\n", inf_frames ? (sum_inf / inf_frames) * 1000.0 : 0);

    uartvis::vis_uart_finish();

    if (jsonl_fp) { fclose(jsonl_fp); jsonl_fp = nullptr; }

    cap.close();
    rknn_destroy(ctx);

    if (exit_code != 0) {
        fprintf(stderr, "[main] ★ 异常退出：抓帧自愈失败（exit_code=%d），"
                        "资源已释放但测试未正常完成\n", exit_code);
        return exit_code;
    }
    printf("[main] 资源已释放，正常退出\n");
    return 0;
}
