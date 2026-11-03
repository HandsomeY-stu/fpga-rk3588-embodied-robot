/*
 * yolo_postprocess.hpp
 * YOLO检测解码、坐标映射及非极大值抑制，输出统一目标框。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef BOTTLE_YOLO_POSTPROCESS_HPP
#define BOTTLE_YOLO_POSTPROCESS_HPP

#include "rknn_api.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
static bool check_yolo_model(rknn_context ctx) {
    rknn_input_output_num n={};
    if(rknn_query(ctx,RKNN_QUERY_IN_OUT_NUM,&n,sizeof(n)) || n.n_input!=1 || n.n_output!=3) return false;
    rknn_tensor_attr a={};
    if(rknn_query(ctx,RKNN_QUERY_INPUT_ATTR,&a,sizeof(a))) return false;
    bool ok=a.n_dims==4 && ((a.fmt==RKNN_TENSOR_NHWC && a.dims[1]==640 && a.dims[2]==640 && a.dims[3]==3) || (a.fmt==RKNN_TENSOR_NCHW && a.dims[1]==3 && a.dims[2]==640 && a.dims[3]==640));
    for(int i=0;i<3;i++) {
        a={}; a.index=i;
        if(rknn_query(ctx,RKNN_QUERY_OUTPUT_ATTR,&a,sizeof(a))) return false;
        printf("[yolo] out%d dims=%u,%u,%u,%u fmt=%d elems=%u\n",i,a.dims[0],a.dims[1],a.dims[2],a.dims[3],a.fmt,a.n_elems);
        int g=80>>i;
        ok=ok && a.n_dims==4 && a.dims[0]==1 && a.dims[1]==255 && a.dims[2]==(unsigned)g && a.dims[3]==(unsigned)g && a.fmt==RKNN_TENSOR_NCHW;
    }
    if(!ok) fprintf(stderr,"[yolo] incompatible tensor layout; abort\n");
    return ok;
}
static int yp_x=0, yp_y=0, yp_w=640, yp_h=640;

static float yp_sharpen = 0.0f;

static void usm_sharpen_box(uint8_t* out, int x0, int y0, int w, int h) {
    if (yp_sharpen <= 0.0f || w < 3 || h < 3) return;
    const int W = 640;
    static std::vector<uint8_t> src;
    src.assign(out, out + (size_t)W * 640 * 3);
    const float k = yp_sharpen;
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            for (int c = 0; c < 3; c++) {
                const int idx = ((y0 + y) * W + (x0 + x)) * 3 + c;
                int sum = 0;
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                        sum += src[(size_t)((y0 + y + dy) * W + (x0 + x + dx)) * 3 + c];
                const float blur = sum / 9.0f;
                float v = src[idx] + k * (src[idx] - blur);
                if (v < 0.f) v = 0.f;
                if (v > 255.f) v = 255.f;
                out[idx] = (uint8_t)(v + 0.5f);
            }
        }
    }
}
static void preprocess_yolo(const uint8_t* bgr,int w,int h,int bpl,uint8_t* out) {
    float r=std::min(640.f/w,640.f/h);
    yp_w=std::max(1,(int)std::round(w*r)); yp_h=std::max(1,(int)std::round(h*r));
    yp_x=(640-yp_w)/2; yp_y=(640-yp_h)/2;
    std::fill(out,out+640*640*3,114);
    for(int y=0;y<yp_h;y++) {
        float fy=std::max(0.f,(y+0.5f)*h/yp_h-0.5f); int y0=(int)fy,y1=std::min(y0+1,h-1); float dy=fy-y0;
        for(int x=0;x<yp_w;x++) {
            float fx=std::max(0.f,(x+0.5f)*w/yp_w-0.5f); int x0=(int)fx,x1=std::min(x0+1,w-1); float dx=fx-x0;
            for(int c=0;c<3;c++) {
                float v0=bgr[y0*bpl+x0*3+2-c]*(1-dx)+bgr[y0*bpl+x1*3+2-c]*dx;
                float v1=bgr[y1*bpl+x0*3+2-c]*(1-dx)+bgr[y1*bpl+x1*3+2-c]*dx;
                out[((y+yp_y)*640+x+yp_x)*3+c]=(uint8_t)(v0*(1-dy)+v1*dy+0.5f);
            }
        }
    }
    usm_sharpen_box(out, yp_x, yp_y, yp_w, yp_h);    
}
static std::vector<Detection> decode_yolo(const rknn_output* outs,int w,int h,float threshold,float nms) {
    static const int anchors[3][6]={{10,13,16,30,33,23},{30,61,62,45,59,119},{116,90,156,198,373,326}};

    static const int ids[80]={1,2,3,4,5,6,7,8,9,10,11,13,14,15,16,17,18,19,20,21,22,23,24,25,27,28,31,32,33,34,35,36,37,38,39,40,41,42,43,44,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,67,70,72,73,74,75,76,77,78,79,80,81,82,84,85,86,87,88,89,90};
    std::vector<Detection> v,keep;
    for(int s=0;s<3;s++) {
        int g=80>>s, len=g*g, stride=8<<s;
        if(!outs[s].buf || outs[s].size < 255*len*sizeof(float)) return keep;
        const float* p=(const float*)outs[s].buf;
        for(int a=0;a<3;a++) for(int y=0;y<g;y++) for(int x=0;x<g;x++) {
            int off=a*85*len+y*g+x; float obj=p[off+4*len]; if(obj<threshold) continue;
            int cls=0; float score=p[off+5*len];
            for(int c=1;c<80;c++) if(p[off+(5+c)*len]>score) {score=p[off+(5+c)*len];cls=c;}
            score*=obj; if(!std::isfinite(score)||score<threshold) continue;
            float cx=(p[off]*2-0.5f+x)*stride,cy=(p[off+len]*2-0.5f+y)*stride;
            float bw=std::pow(p[off+2*len]*2,2)*anchors[s][2*a],bh=std::pow(p[off+3*len]*2,2)*anchors[s][2*a+1];
            Detection d={}; d.cls=ids[cls];d.conf=score;
            d.xmin=std::max(0.f,std::min(1.f,(cx-bw/2-yp_x)/yp_w));d.xmax=std::max(0.f,std::min(1.f,(cx+bw/2-yp_x)/yp_w));
            d.ymin=std::max(0.f,std::min(1.f,(cy-bh/2-yp_y)/yp_h));d.ymax=std::max(0.f,std::min(1.f,(cy+bh/2-yp_y)/yp_h));
            if(d.xmax<=d.xmin || d.ymax<=d.ymin)continue;
            d.px=(d.xmin+d.xmax)*w/2;d.py=(d.ymin+d.ymax)*h/2;d.pw=(d.xmax-d.xmin)*w;d.ph=(d.ymax-d.ymin)*h;v.push_back(d);
        }
    }
    std::sort(v.begin(),v.end(),[](const Detection&a,const Detection&b){return a.conf>b.conf;});
    for(const auto&d:v) {
        bool reject=false;
        for(const auto&k:keep) if(k.cls==d.cls) {
            float inter=std::max(0.f,std::min(k.xmax,d.xmax)-std::max(k.xmin,d.xmin))*std::max(0.f,std::min(k.ymax,d.ymax)-std::max(k.ymin,d.ymin));
            float total=(k.xmax-k.xmin)*(k.ymax-k.ymin)+(d.xmax-d.xmin)*(d.ymax-d.ymin)-inter;
            if(total>0 && inter/total>nms){reject=true;break;}
        }
        if(!reject)keep.push_back(d);
        if(keep.size()>=100)break;
    }
    return keep;
}

struct RedBlob { int cx, cy, w, h, n; };
// 网格连通区域统计寻找主要红色区域；只提供颜色方位，不独立确认目标身份。
static RedBlob find_red_blob(const uint8_t* bgr, int w, int h, int bpl) {
    RedBlob rb{0,0,0,0,0};
    const int GW=32, GH=18;
    static int cnt[GH][GW];
    for (int gy=0; gy<GH; gy++) for (int gx=0; gx<GW; gx++) cnt[gy][gx]=0;
    int total=0;
    for (int y=0; y<h; y+=4) for (int x=0; x<w; x+=4) {
        const uint8_t* q = bgr + (size_t)y*bpl + (size_t)x*3;
        int b=q[0], g=q[1], r=q[2];
        if (r>=35 && 100*r>135*g && 100*r>125*b && r-std::min(g,b)>25) {
            int gx=std::min(GW-1, x*GW/w), gy=std::min(GH-1, y*GH/h);
            cnt[gy][gx]++; total++;
        }
    }
    if (total < 25) return rb;                  
    int bgx=0, bgy=0, best=0;
    for (int gy=0; gy<GH; gy++) for (int gx=0; gx<GW; gx++)
        if (cnt[gy][gx] > best) { best=cnt[gy][gx]; bgx=gx; bgy=gy; }
    if (best < 8) return rb;                    
    int gx0=std::max(0,bgx-2), gx1=std::min(GW-1,bgx+2);
    int gy0=std::max(0,bgy-2), gy1=std::min(GH-1,bgy+2);
    long sx=0, sy=0, n=0;
    int x0=w, x1=0, y0=h, y1=0;
    for (int y=gy0*h/GH; y<=(gy1+1)*h/GH && y<h; y+=4)
        for (int x=gx0*w/GW; x<=(gx1+1)*w/GW && x<w; x+=4) {
            const uint8_t* q = bgr + (size_t)y*bpl + (size_t)x*3;
            int b=q[0], g=q[1], r=q[2];
            if (r>=35 && 100*r>135*g && 100*r>125*b && r-std::min(g,b)>25) {
                sx+=x; sy+=y; n++;
                if(x<x0)x0=x; if(x>x1)x1=x; if(y<y0)y0=y; if(y>y1)y1=y;
            }
        }
    if (n < 20) return rb;
    rb.cx=(int)(sx/n); rb.cy=(int)(sy/n);
    rb.w=x1-x0; rb.h=y1-y0; rb.n=(int)n;
    return rb;
}

struct RedHalf { int cx, n; };
// 左右半幅分别统计，避免最大红块掩盖另一侧较小红瓶的方位信息。
static void find_red_halves(const uint8_t* bgr, int w, int h, int bpl,
                            RedHalf &left, RedHalf &right) {
    left = {0, 0}; right = {0, 0};
    long lx = 0, ln = 0, rx = 0, rn = 0;
    int mid = w / 2;
    for (int y = 0; y < h; y += 4) {
        for (int x = 0; x < w; x += 4) {
            const uint8_t* q = bgr + (size_t)y * bpl + (size_t)x * 3;
            int b = q[0], g = q[1], r = q[2];
            if (r >= 35 && 100*r > 135*g && 100*r > 125*b && r - std::min(g,b) > 25) {
                if (x < mid) { lx += x; ln++; }
                else         { rx += x; rn++; }
            }
        }
    }
    if (ln > 0) { left.cx = (int)(lx / ln); left.n = (int)ln; }
    if (rn > 0) { right.cx = (int)(rx / rn); right.n = (int)rn; }
}
static float bottle_red_ratio(const uint8_t* bgr,int w,int h,int bpl,const Detection& d) {
    int x0=std::max(0,(int)((d.xmin+.15f*(d.xmax-d.xmin))*w));
    int x1=std::min(w,(int)((d.xmax-.15f*(d.xmax-d.xmin))*w));
    int y0=std::max(0,(int)((d.ymin+.12f*(d.ymax-d.ymin))*h));
    int y1=std::min(h,(int)((d.ymax-.08f*(d.ymax-d.ymin))*h));
    int red=0,total=0;
    for(int y=y0;y<y1;y+=3)for(int x=x0;x<x1;x+=3){
        const uint8_t* q=bgr+y*bpl+x*3;int b=q[0],g=q[1],r=q[2];++total;
        if(r>=35 && 100*r>135*g && 100*r>125*b && r-std::min(g,b)>25)++red;
    }
    return total?float(red)/total:0.f;
}
#endif
