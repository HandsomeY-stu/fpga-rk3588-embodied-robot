/*
 * ssd_postprocess.cpp
 * 检测数据结构与SSD兼容后处理，供实时推理主程序共同使用。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#include "ssd_postprocess.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

bool SSDPostProcess::loadAnchors(const std::string &box_priors_path,
                                 const std::string &labels_path) {
    m_anchors.clear();
    std::ifstream f(box_priors_path);
    if (!f.is_open()) {
        fprintf(stderr, "[ssd] 打不开 box_priors: %s\n", box_priors_path.c_str());
        return false;
    }

    std::vector<std::vector<float>> rows;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line);
        std::vector<float> vals;
        float v;
        while (iss >> v) vals.push_back(v);
        if (!vals.empty()) rows.push_back(std::move(vals));
    }

    if (rows.size() != 4) {
        fprintf(stderr, "[ssd] box_priors 应为 4 行(cx,cy,w,h)，实际 %zu 行\n",
                rows.size());
        return false;
    }
    size_t n = rows[0].size();
    for (size_t r = 1; r < 4; r++) {
        if (rows[r].size() != n) {
            fprintf(stderr, "[ssd] box_priors 行 %zu 长度 %zu != 行0 的 %zu\n",
                    r, rows[r].size(), n);
            return false;
        }
    }

    m_anchors.resize(n * 4);
    for (size_t i = 0; i < n; i++) {
        m_anchors[i * 4 + 0] = rows[0][i];    
        m_anchors[i * 4 + 1] = rows[1][i];    
        m_anchors[i * 4 + 2] = rows[2][i];    
        m_anchors[i * 4 + 3] = rows[3][i];    
    }
    printf("[ssd] 载入 anchors: %zu 个（box_priors 为 4行×%zu列，已转置为 cx,cy,w,h）\n",
           n, n);

    if (!labels_path.empty()) {
        std::ifstream lf(labels_path);
        std::string ln;
        while (std::getline(lf, ln)) {
            while (!ln.empty() && (ln.back() == '\r' || ln.back() == '\n' ||
                                   ln.back() == ' ' || ln.back() == '\t'))
                ln.pop_back();
            size_t s = ln.find_first_not_of(" \t");
            if (s != std::string::npos) ln = ln.substr(s);
            if (!ln.empty()) m_labels.push_back(ln);
        }
        printf("[ssd] 载入类别名: %zu 条\n", m_labels.size());
    }
    return true;
}

const std::string &SSDPostProcess::labelOf(int cls) const {
    if (cls >= 0 && cls < (int)m_labels.size()) return m_labels[cls];
    return m_empty;
}

static float s_lb_padx = 0.f, s_lb_pady = 0.f;
static float s_lb_w = 300.f, s_lb_h = 300.f;
static bool  s_lb_on = false;

static bool  s_clahe_on = false;
static float s_clahe_clip = 3.0f;

void set_clahe(bool on, float clip) { s_clahe_on = on; s_clahe_clip = (clip > 0.5f ? clip : 3.0f); }

std::vector<Detection> SSDPostProcess::decode(const float *loc,
                                              const float *conf,
                                              int img_w, int img_h,
                                              float conf_thresh,
                                              float nms_thresh) const {
    std::vector<Detection> dets;
    const size_t n = anchorCount();
    if (n == 0) return dets;

    const int NCLS = 91;

    for (size_t i = 0; i < n; i++) {

        const float *cp = conf + i * NCLS;
        float maxv = cp[0];
        for (int c = 1; c < NCLS; c++) if (cp[c] > maxv) maxv = cp[c];

        float sum = 0.f;
        for (int c = 0; c < NCLS; c++) sum += expf(cp[c] - maxv);
        if (sum <= 0.f) continue;

        int   best_c = -1;
        float best_p = 0.f;
        for (int c = 1; c < NCLS; c++) {    
            float p = expf(cp[c] - maxv) / sum;
            if (p > best_p) { best_p = p; best_c = c; }
        }
        if (best_c < 0 || best_p < conf_thresh) continue;

        const float acx = m_anchors[i * 4 + 0];
        const float acy = m_anchors[i * 4 + 1];
        const float aw  = m_anchors[i * 4 + 2];
        const float ah  = m_anchors[i * 4 + 3];

        const float VAR0 = 0.1f, VAR1 = 0.2f;
        const float dx = loc[i * 4 + 0] * VAR0;
        const float dy = loc[i * 4 + 1] * VAR0;
        const float dw = loc[i * 4 + 2] * VAR1;
        const float dh = loc[i * 4 + 3] * VAR1;

        Detection d;
        d.cls  = best_c;
        d.conf = best_p;

        float bcx = acx + dx * aw;
        float bcy = acy + dy * ah;
        float bw  = aw * expf(dw);
        float bh  = ah * expf(dh);

        d.xmin = bcx - 0.5f * bw;
        d.ymin = bcy - 0.5f * bh;
        d.xmax = bcx + 0.5f * bw;
        d.ymax = bcy + 0.5f * bh;

        if (s_lb_on) {
            d.xmin = (d.xmin * 300.f - s_lb_padx) / s_lb_w;
            d.xmax = (d.xmax * 300.f - s_lb_padx) / s_lb_w;
            d.ymin = (d.ymin * 300.f - s_lb_pady) / s_lb_h;
            d.ymax = (d.ymax * 300.f - s_lb_pady) / s_lb_h;
        }

        if (d.xmin < 0.f) d.xmin = 0.f;
        if (d.ymin < 0.f) d.ymin = 0.f;
        if (d.xmax > 1.f) d.xmax = 1.f;
        if (d.ymax > 1.f) d.ymax = 1.f;
        if (d.xmax <= d.xmin || d.ymax <= d.ymin) continue;

        d.px = 0.5f * (d.xmin + d.xmax) * img_w;
        d.py = 0.5f * (d.ymin + d.ymax) * img_h;
        d.pw = (d.xmax - d.xmin) * img_w;
        d.ph = (d.ymax - d.ymin) * img_h;

        dets.push_back(d);
    }

    std::sort(dets.begin(), dets.end(),
              [](const Detection &a, const Detection &b) { return a.conf > b.conf; });

    if (dets.empty()) return dets;

    std::vector<int> keep;
    std::vector<char> removed(dets.size(), 0);
    for (size_t i = 0; i < dets.size(); i++) {
        if (removed[i]) continue;
        keep.push_back((int)i);
        for (size_t j = i + 1; j < dets.size(); j++) {
            if (removed[j]) continue;
            if (dets[j].cls != dets[i].cls) continue;

            float ix1 = std::max(dets[i].xmin, dets[j].xmin);
            float iy1 = std::max(dets[i].ymin, dets[j].ymin);
            float ix2 = std::min(dets[i].xmax, dets[j].xmax);
            float iy2 = std::min(dets[i].ymax, dets[j].ymax);
            float iw  = ix2 - ix1, ih = iy2 - iy1;
            if (iw <= 0.f || ih <= 0.f) continue;

            float inter = iw * ih;
            float ai = (dets[i].xmax - dets[i].xmin) * (dets[i].ymax - dets[i].ymin);
            float aj = (dets[j].xmax - dets[j].xmin) * (dets[j].ymax - dets[j].ymin);
            float uni = ai + aj - inter;
            float iou = uni > 0.f ? inter / uni : 0.f;
            if (iou > nms_thresh) removed[j] = 1;
        }
    }

    std::vector<Detection> out;
    out.reserve(keep.size());
    for (int k : keep) out.push_back(dets[k]);
    return out;
}

static float s_gain_r = 1.0f;
static float s_gain_g = 1.0f;
static float s_gain_b = 1.0f;

void set_color_gain(float gain_r, float gain_g, float gain_b) {
    s_gain_r = (gain_r > 0.0f) ? gain_r : 1.0f;
    s_gain_g = (gain_g > 0.0f) ? gain_g : 1.0f;
    s_gain_b = (gain_b > 0.0f) ? gain_b : 1.0f;
}

void get_color_gain(float &gain_r, float &gain_g, float &gain_b) {
    gain_r = s_gain_r; gain_g = s_gain_g; gain_b = s_gain_b;
}

void preprocess_bgr24_to_300(const uint8_t *bgr, int src_w, int src_h,
                             int src_bpl, uint8_t *out_rgb_300) {
    const int DW = 300, DH = 300;

    const bool identity = (s_gain_r == 1.0f && s_gain_g == 1.0f && s_gain_b == 1.0f);
    uint8_t lut_r[256], lut_g[256], lut_b[256];
    if (!identity) {
        for (int i = 0; i < 256; i++) {
            int vr = (int)(i * s_gain_r + 0.5f);
            int vg = (int)(i * s_gain_g + 0.5f);
            int vb = (int)(i * s_gain_b + 0.5f);
            lut_r[i] = (uint8_t)(vr < 0 ? 0 : (vr > 255 ? 255 : vr));
            lut_g[i] = (uint8_t)(vg < 0 ? 0 : (vg > 255 ? 255 : vg));
            lut_b[i] = (uint8_t)(vb < 0 ? 0 : (vb > 255 ? 255 : vb));
        }
    }

    float sc = (float)DW / src_w;
    if ((float)DH / src_h < sc) sc = (float)DH / src_h;
    int w2 = (int)(src_w * sc + 0.5f); if (w2 < 1) w2 = 1; if (w2 > DW) w2 = DW;
    int h2 = (int)(src_h * sc + 0.5f); if (h2 < 1) h2 = 1; if (h2 > DH) h2 = DH;
    int px0 = (DW - w2) / 2, py0 = (DH - h2) / 2;
    s_lb_padx = (float)px0; s_lb_pady = (float)py0;
    s_lb_w = (float)w2;     s_lb_h = (float)h2;
    s_lb_on  = !(w2 == DW && h2 == DH);

    for (int i = 0; i < DW * DH; i++) {
        out_rgb_300[i * 3 + 0] = 114;
        out_rgb_300[i * 3 + 1] = 114;
        out_rgb_300[i * 3 + 2] = 114;
    }

    static const int CT = 8;
    static uint32_t chist[8][8][256];
    static uint8_t  clut[8][8][256];
    static uint16_t ktab[8][8][256];    
    if (s_clahe_on) {
        for (int i = 0; i < CT; i++)
            for (int j = 0; j < CT; j++)
                for (int v = 0; v < 256; v++) chist[i][j][v] = 0;
        for (int yy = 0; yy < src_h; yy += 2) {
            const uint8_t *r2 = bgr + (size_t)yy * src_bpl;
            int ti = (int)((long)yy * CT / src_h); if (ti >= CT) ti = CT - 1;
            for (int xx = 0; xx < src_w; xx += 2) {
                const uint8_t *pp = r2 + (size_t)xx * 3;
                int lum = (pp[0] * 29 + pp[1] * 150 + pp[2] * 77) >> 8;   
                int tj = (int)((long)xx * CT / src_w); if (tj >= CT) tj = CT - 1;
                chist[ti][tj][lum]++;
            }
        }
        for (int i = 0; i < CT; i++)
            for (int j = 0; j < CT; j++) {
                uint32_t total = 0;
                for (int v = 0; v < 256; v++) total += chist[i][j][v];
                if (total == 0) { for (int v = 0; v < 256; v++) clut[i][j][v] = (uint8_t)v; continue; }
                uint32_t lim = (uint32_t)(s_clahe_clip * (float)total / 256.0f);
                if (lim < 1) lim = 1;
                uint32_t acc = 0, sum2 = 0;
                for (int v = 0; v < 256; v++) {
                    uint32_t h = chist[i][j][v];
                    if (h > lim) h = lim;
                    sum2 += h;
                }
                if (sum2 == 0) { for (int v = 0; v < 256; v++) clut[i][j][v] = (uint8_t)v; continue; }
                for (int v = 0; v < 256; v++) {
                    uint32_t h = chist[i][j][v];
                    if (h > lim) h = lim;
                    acc += h;
                    clut[i][j][v] = (uint8_t)((acc * 255u) / sum2);
                }
            }

        for (int i = 0; i < CT; i++)
            for (int j = 0; j < CT; j++)
                for (int v = 0; v < 256; v++) {
                    if (v > 4) {
                        uint32_t kk = ((uint32_t)clut[i][j][v] << 8) / (uint32_t)v;
                        if (kk > 1024) kk = 1024;
                        ktab[i][j][v] = (uint16_t)kk;
                    } else {
                        ktab[i][j][v] = 256;
                    }
                }
    }

    static std::vector<uint8_t> s_clahe_buf;
    const uint8_t *src_base = bgr;
    int            src_bpl_eff = src_bpl;
    if (s_clahe_on) {
        size_t need = (size_t)src_w * src_h * 3;
        if (s_clahe_buf.size() < need) s_clahe_buf.resize(need);
        int rowT[8];
        for (int i = 0; i < CT; i++) {
            int y0 = (int)((long)i * src_h / CT);
            rowT[i] = y0;
        }
        for (int yy = 0; yy < src_h; yy++) {
            int ti = (int)((long)yy * CT / src_h); if (ti >= CT) ti = CT - 1;
            const uint8_t *r2 = bgr + (size_t)yy * src_bpl;
            uint8_t *o2 = s_clahe_buf.data() + (size_t)yy * (size_t)src_w * 3;
            for (int xx = 0; xx < src_w; xx++) {
                const uint8_t *pp = r2 + (size_t)xx * 3;
                int lum = (pp[0] * 29 + pp[1] * 150 + pp[2] * 77) >> 8;
                int tj = (int)((long)xx * CT / src_w); if (tj >= CT) tj = CT - 1;
                int b2 = pp[0], g2 = pp[1], r3 = pp[2];
                uint32_t kk = ktab[ti][tj][lum];           
                int t;
                t = (b2 * (int)kk) >> 8; b2 = t > 255 ? 255 : t;
                t = (g2 * (int)kk) >> 8; g2 = t > 255 ? 255 : t;
                t = (r3 * (int)kk) >> 8; r3 = t > 255 ? 255 : t;
                o2[xx * 3 + 0] = (uint8_t)b2;
                o2[xx * 3 + 1] = (uint8_t)g2;
                o2[xx * 3 + 2] = (uint8_t)r3;
            }
        }
        src_base = s_clahe_buf.data();
        src_bpl_eff = src_w * 3;
    }

    for (int dy = 0; dy < h2; dy++) {
        int sy = (int)((float)dy * src_h / h2);
        if (sy >= src_h) sy = src_h - 1;
        const uint8_t *row = src_base + (size_t)sy * src_bpl_eff;
        for (int dx = 0; dx < w2; dx++) {
            int sx = (int)((float)dx * src_w / w2);
            if (sx >= src_w) sx = src_w - 1;
            const uint8_t *p = row + (size_t)sx * 3;
            uint8_t *o = out_rgb_300 + ((size_t)(dy + py0) * DW + (dx + px0)) * 3;

            uint8_t R = p[2], G = p[1], B = p[0];
            if (!identity) { R = lut_r[R]; G = lut_g[G]; B = lut_b[B]; }
            o[0] = R;
            o[1] = G;
            o[2] = B;
        }
    }

}

void filter_dets_in_rect(std::vector<Detection> &dets,
                         int x0, int y0, int x1, int y1) {
    if (x1 <= x0 || y1 <= y0) return;
    std::vector<Detection> keep;
    keep.reserve(dets.size());
    for (size_t i = 0; i < dets.size(); i++) {
        const Detection &d = dets[i];
        bool inside = (d.px >= (float)x0 && d.px <= (float)x1 &&
                       d.py >= (float)y0 && d.py <= (float)y1);
        if (!inside) keep.push_back(d);
    }
    dets.swap(keep);
}

void draw_boxes_bgr(uint8_t *bgr, int w, int h, int bpl,
                    const std::vector<Detection> &dets) {
    auto setpx = [&](int x, int y, uint8_t b, uint8_t g, uint8_t r) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        uint8_t *p = bgr + (size_t)y * bpl + (size_t)x * 3;
        p[0] = b; p[1] = g; p[2] = r;
    };
    auto hline = [&](int x0, int x1, int y, uint8_t b, uint8_t g, uint8_t r) {
        for (int x = x0; x <= x1; x++) setpx(x, y, b, g, r);
    };
    auto vline = [&](int y0, int y1, int x, uint8_t b, uint8_t g, uint8_t r) {
        for (int y = y0; y <= y1; y++) setpx(x, y, b, g, r);
    };

    for (const auto &d : dets) {
        int x0 = (int)(d.xmin * w), x1 = (int)(d.xmax * w);
        int y0 = (int)(d.ymin * h), y1 = (int)(d.ymax * h);

        uint8_t b = (uint8_t)((d.cls * 67)  & 0xFF);
        uint8_t g = (uint8_t)((d.cls * 131) & 0xFF);
        uint8_t r = (uint8_t)((d.cls * 199) & 0xFF);
        for (int t = 0; t < 3; t++) {           
            hline(x0, x1, y0 + t, b, g, r);
            hline(x0, x1, y1 - t, b, g, r);
            vline(y0, y1, x0 + t, b, g, r);
            vline(y0, y1, x1 - t, b, g, r);
        }
    }
}

struct Glyph5x7 { uint8_t ch; uint8_t row[7]; };

static const Glyph5x7 FONT5X7[] = {
    {0x20, {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},   
    {0x2B, {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}},   
    {0x2D, {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},   
    {0x2E, {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}},   
    {0x30, {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}},   
    {0x31, {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}},   
    {0x32, {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}},   
    {0x33, {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E}},   
    {0x34, {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}},   
    {0x35, {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}},   
    {0x36, {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}},   
    {0x37, {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},   
    {0x38, {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}},   
    {0x39, {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}},   
    {0x3A, {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}},   
    {0x41, {0x04,0x0A,0x11,0x11,0x1F,0x11,0x11}},   
    {0x42, {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}},   
    {0x43, {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},   
    {0x44, {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}},   
    {0x45, {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},   
    {0x46, {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}},   
    {0x47, {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E}},   
    {0x48, {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},   
    {0x49, {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}},   
    {0x4A, {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}},   
    {0x4B, {0x11,0x12,0x14,0x18,0x14,0x12,0x11}},   
    {0x4C, {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},   
    {0x4D, {0x11,0x1B,0x15,0x11,0x11,0x11,0x11}},   
    {0x4E, {0x11,0x19,0x15,0x13,0x11,0x11,0x11}},   
    {0x4F, {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},   
    {0x50, {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}},   
    {0x51, {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}},   
    {0x52, {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},   
    {0x53, {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}},   
    {0x54, {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},   
    {0x55, {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}},   
    {0x56, {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}},   
    {0x57, {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}},   
    {0x58, {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}},   
    {0x59, {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}},   
    {0x5A, {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}},   
};

static const uint8_t *glyph5x7_of(uint8_t c) {
    if (c >= 'a' && c <= 'z') c = (uint8_t)(c - 'a' + 'A');
    for (size_t i = 0; i < sizeof(FONT5X7) / sizeof(FONT5X7[0]); i++)
        if (FONT5X7[i].ch == c) return FONT5X7[i].row;
    return nullptr;
}

static inline void put_px(uint8_t *bgr, int w, int h, int bpl, int x, int y,
                          uint8_t b, uint8_t g, uint8_t r) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    uint8_t *p = bgr + (size_t)y * bpl + (size_t)x * 3;
    p[0] = b; p[1] = g; p[2] = r;
}

static void fill_rect(uint8_t *bgr, int w, int h, int bpl,
                      int x0, int y0, int x1, int y1,
                      uint8_t b, uint8_t g, uint8_t r) {
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) put_px(bgr, w, h, bpl, x, y, b, g, r);
}

void draw_text_bgr(uint8_t *bgr, int w, int h, int bpl, int x, int y,
                   const char *s, int scale,
                   uint8_t b, uint8_t g, uint8_t r, bool bg_black) {
    if (!s || scale < 1) return;
    int n = 0;
    while (s[n]) n++;
    if (n == 0) return;
    const int adv = 6 * scale;                  
    if (bg_black)
        fill_rect(bgr, w, h, bpl, x - 2, y - 2,
                  x + n * adv + 1, y + 7 * scale + 1, 0, 0, 0);
    int cx = x;
    for (int i = 0; i < n; i++) {
        const uint8_t *gl = glyph5x7_of((uint8_t)s[i]);
        if (gl) {
            for (int row = 0; row < 7; row++) {
                uint8_t bits = gl[row];
                for (int col = 0; col < 5; col++) {
                    if (!(bits & (uint8_t)(1 << (4 - col)))) continue;
                    for (int dy = 0; dy < scale; dy++)
                        for (int dx = 0; dx < scale; dx++)
                            put_px(bgr, w, h, bpl, cx + col * scale + dx,
                                   y + row * scale + dy, b, g, r);
                }
            }
        }
        cx += adv;
    }
}

void draw_boxes_yellow_labeled(uint8_t *bgr, int w, int h, int bpl,
                               const std::vector<Detection> &dets,
                               const std::vector<std::string> *names,
                               int scale) {
    const uint8_t YB = 0, YG = 255, YR = 255;    
    if (scale < 1) scale = 1;
    for (size_t i = 0; i < dets.size(); i++) {
        const Detection &d = dets[i];
        int x0 = (int)(d.xmin * w), x1 = (int)(d.xmax * w);
        int y0 = (int)(d.ymin * h), y1 = (int)(d.ymax * h);
        if (x1 < x0) { int t = x0; x0 = x1; x1 = t; }
        if (y1 < y0) { int t = y0; y0 = y1; y1 = t; }
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 > w - 1) x1 = w - 1;
        if (y1 > h - 1) y1 = h - 1;
        if (x1 <= x0 || y1 <= y0) continue;

        for (int t = 0; t < 3; t++) {
            fill_rect(bgr, w, h, bpl, x0, y0 + t, x1, y0 + t, YB, YG, YR);
            fill_rect(bgr, w, h, bpl, x0, y1 - t, x1, y1 - t, YB, YG, YR);
            fill_rect(bgr, w, h, bpl, x0 + t, y0, x0 + t, y1, YB, YG, YR);
            fill_rect(bgr, w, h, bpl, x1 - t, y0, x1 - t, y1, YB, YG, YR);
        }

        if (!names || d.cls < 0 || d.cls >= (int)names->size()) continue;
        const std::string &nm = (*names)[d.cls];
        if (nm.empty()) continue;

        int ty = y1 + 4;
        if (ty + 7 * scale + 2 > h) ty = y0 - 7 * scale - 4;
        if (ty < 0) ty = y0 + 2;
        int tx = x0;
        int tw = (int)nm.size() * 6 * scale;
        if (tx + tw > w) tx = w - tw - 2;
        if (tx < 0) tx = 0;
        draw_text_bgr(bgr, w, h, bpl, tx, ty, nm.c_str(), scale,
                      YB, YG, YR, true);
    }
}

bool save_bmp_bgr24(const char *path, const uint8_t *bgr, int w, int h, int bpl) {
    FILE *f = fopen(path, "wb");
    if (!f) return false;

    const int row_raw   = w * 3;
    const int row_pad   = (4 - (row_raw % 4)) % 4;
    const int row_total = row_raw + row_pad;
    const uint32_t img_size  = (uint32_t)row_total * h;
    const uint32_t file_size = 14 + 40 + img_size;

    uint8_t fh[14] = {0};
    fh[0] = 'B'; fh[1] = 'M';
    memcpy(fh + 2, &file_size, 4);
    uint32_t off = 14 + 40;
    memcpy(fh + 10, &off, 4);
    fwrite(fh, 1, 14, f);

    uint8_t ih[40] = {0};
    uint32_t hs = 40;
    memcpy(ih + 0,  &hs, 4);
    int32_t  wi = w, hi = h;
    memcpy(ih + 4,  &wi, 4);
    memcpy(ih + 8,  &hi, 4);
    uint16_t planes = 1, bpp = 24;
    memcpy(ih + 12, &planes, 2);
    memcpy(ih + 14, &bpp, 2);
    memcpy(ih + 20, &img_size, 4);
    int32_t ppm = 2835;
    memcpy(ih + 24, &ppm, 4);
    memcpy(ih + 28, &ppm, 4);
    fwrite(ih, 1, 40, f);

    uint8_t pad[3] = {0, 0, 0};
    for (int y = h - 1; y >= 0; y--) {      
        const uint8_t *src = bgr + (size_t)y * bpl;
        fwrite(src, 1, row_raw, f);
        if (row_pad) fwrite(pad, 1, row_pad, f);
    }
    fclose(f);
    return true;
}
