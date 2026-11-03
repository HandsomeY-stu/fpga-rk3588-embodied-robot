/*
 * test_yolo.cpp
 * 实际C++后处理单元测试，不访问摄像头或电机。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#include "ssd_postprocess.hpp"
#include "yolo_postprocess.hpp"
#include <cassert>
#include <iostream>
int main(){
 std::vector<uint8_t> raw(1280*720*3),out(640*640*3);
 for(size_t i=0;i<raw.size();i+=3){raw[i]=10;raw[i+1]=20;raw[i+2]=30;}
 preprocess_yolo(raw.data(),1280,720,1280*3,out.data());
 assert(yp_x==0 && yp_y==140 && yp_w==640 && yp_h==360);
 assert(out[0]==114 && out[140*640*3]==30 && out[140*640*3+2]==10);
 std::vector<float> buf[3];rknn_output o[3]={};
 for(int s=0;s<3;s++){int g=80>>s;buf[s].resize(255*g*g);o[s].buf=buf[s].data();o[s].size=buf[s].size()*4;}
 assert(decode_yolo(o,1280,720,.4,.45).empty());
 int len=80*80,off=40*80+40;
 auto put=[&](int anchor,int cls,float obj,float cp){int k=off+anchor*85*len;buf[0][k]=.5;buf[0][k+len]=.5;buf[0][k+2*len]=.5;buf[0][k+3*len]=.5;buf[0][k+4*len]=obj;buf[0][k+(5+cls)*len]=cp;};
 put(0,39,.9,.9);auto d=decode_yolo(o,1280,720,.4,.45);
 assert(d.size()==1 && d[0].cls==44 && std::abs(d[0].conf-.81)<.0001 && std::abs(d[0].px-648)<.01 && std::abs(d[0].py-368)<.01);
 put(0,39,.5,.5);assert(decode_yolo(o,1280,720,.4,.45).empty());
 put(0,39,.9,.9);put(1,39,.8,.8);

 buf[0][off+85*len+2*len]=.5*std::sqrt(10.f/16);buf[0][off+85*len+3*len]=.5*std::sqrt(13.f/30);
 assert(decode_yolo(o,1280,720,.4,.45).size()==1);
 buf[0][off+85*len+(5+39)*len]=0;buf[0][off+85*len+5*len]=.8;
 assert(decode_yolo(o,1280,720,.4,.45).size()==2);
 Detection box={};box.xmin=0;box.ymin=0;box.xmax=1;box.ymax=1;
 assert(bottle_red_ratio(raw.data(),1280,720,1280*3,box)==0);
 for(size_t i=0;i<raw.size();i+=3){raw[i]=30;raw[i+1]=40;raw[i+2]=100;}
 assert(bottle_red_ratio(raw.data(),1280,720,1280*3,box)>.99);
 for(size_t i=0;i<raw.size();i++)raw[i]=150;
 assert(bottle_red_ratio(raw.data(),1280,720,1280*3,box)==0);
 std::cout<<"PASS same-frame red ratio: neutral, low-intensity red, gray wall\n";
 std::cout<<"PASS RGB/letterbox, empty, class44, inverse coordinates, confidence product, same-class NMS, cross-class retention\n";
}
