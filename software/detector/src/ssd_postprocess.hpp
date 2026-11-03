/*
 * ssd_postprocess.hpp
 * 检测数据结构与SSD兼容后处理，供实时推理主程序共同使用。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef SSD_POSTPROCESS_HPP
#define SSD_POSTPROCESS_HPP

#include <stdint.h>
#include <vector>
#include <string>

struct Detection {
    int    cls;         
    float  conf;
    float  xmin, ymin, xmax, ymax;    

    float  px, py, pw, ph;            
};

class SSDPostProcess {
public:

    bool loadAnchors(const std::string &box_priors_path,
                     const std::string &labels_path = "");

    std::vector<Detection> decode(const float *loc, const float *conf,
                                  int img_w, int img_h,
                                  float conf_thresh = 0.40f,
                                  float nms_thresh  = 0.45f) const;

    const std::string &labelOf(int cls) const;
    const std::vector<std::string> &labels() const { return m_labels; }
    size_t anchorCount() const { return m_anchors.size() / 4; }

private:
    std::vector<float>       m_anchors;    
    std::vector<std::string> m_labels;
    std::string              m_empty;
};

void set_color_gain(float gain_r, float gain_g, float gain_b);
void get_color_gain(float &gain_r, float &gain_g, float &gain_b);

void preprocess_bgr24_to_300(const uint8_t *bgr, int src_w, int src_h,
                             int src_bpl, uint8_t *out_rgb_300);

void set_clahe(bool on, float clip = 3.0f);

void filter_dets_in_rect(std::vector<Detection> &dets,
                         int x0, int y0, int x1, int y1);

void draw_boxes_bgr(uint8_t *bgr, int w, int h, int bpl,
                    const std::vector<Detection> &dets);

void draw_boxes_yellow_labeled(uint8_t *bgr, int w, int h, int bpl,
                               const std::vector<Detection> &dets,
                               const std::vector<std::string> *names,
                               int scale = 2);

void draw_text_bgr(uint8_t *bgr, int w, int h, int bpl, int x, int y,
                   const char *s, int scale,
                   uint8_t b, uint8_t g, uint8_t r, bool bg_black = true);

bool save_bmp_bgr24(const char *path, const uint8_t *bgr,
                    int w, int h, int bpl);

#endif  
