/*
 * abi_check.cpp
 * RKNN结构体ABI布局自检，检查大小与字段偏移。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include "rknn_api.h"

static int g_fail = 0;

#define CHK(expr, msg) do {                                   \
    if (expr) { printf("  [ OK ] %s\n", msg); }               \
    else      { printf("  [FAIL] %s\n", msg); g_fail++; }     \
} while (0)

#define SHOW(type, field) \
    printf("        %-18s offset=%-4zu size=%zu\n", #field, offsetof(type, field), sizeof(((type*)0)->field))

int main() {
    printf("========================================================\n");
    printf(" RKNN ABI 布局自校验\n");
    printf("========================================================\n");

    printf("\n[rknn_context]\n");
    CHK(sizeof(rknn_context) == 8, "sizeof == 8（必须 64 位；32 位会导致 rknn_init 越界段错误）");

    printf("\n[rknn_output]  ← 本轮修正项\n");
    printf("      sizeof = %zu\n", sizeof(rknn_output));
    SHOW(rknn_output, want_float);
    SHOW(rknn_output, is_prealloc);
    SHOW(rknn_output, index);
    SHOW(rknn_output, buf);
    SHOW(rknn_output, size);
    CHK(sizeof(rknn_output) == 24,                            "sizeof == 24");
    CHK(offsetof(rknn_output, want_float)  == 0,              "want_float  offset == 0");
    CHK(offsetof(rknn_output, is_prealloc) == 1,              "is_prealloc offset == 1");
    CHK(offsetof(rknn_output, index)       == 4,              "index       offset == 4  ★旧版错为 0");
    CHK(offsetof(rknn_output, buf)         == 8,              "buf         offset == 8");
    CHK(offsetof(rknn_output, size)        == 16,             "size        offset == 16");

    printf("\n[rknn_input]\n");
    printf("      sizeof = %zu\n", sizeof(rknn_input));
    SHOW(rknn_input, index);
    SHOW(rknn_input, buf);
    SHOW(rknn_input, size);
    SHOW(rknn_input, pass_through);
    SHOW(rknn_input, type);
    SHOW(rknn_input, fmt);
    CHK(sizeof(rknn_input) == 32,                    "sizeof == 32");
    CHK(offsetof(rknn_input, index)        == 0,     "index        offset == 0");
    CHK(offsetof(rknn_input, buf)          == 8,     "buf          offset == 8");
    CHK(offsetof(rknn_input, size)         == 16,    "size         offset == 16");
    CHK(offsetof(rknn_input, pass_through) == 20,    "pass_through offset == 20");
    CHK(offsetof(rknn_input, type)         == 24,    "type         offset == 24");
    CHK(offsetof(rknn_input, fmt)          == 28,    "fmt          offset == 28");

    printf("\n[rknn_tensor_attr]\n");
    printf("      sizeof = %zu  (官方 2022 头文件为 372；本板 runtime 1.6.0 实测要求 376)\n",
           sizeof(rknn_tensor_attr));
    SHOW(rknn_tensor_attr, index);
    SHOW(rknn_tensor_attr, n_dims);
    SHOW(rknn_tensor_attr, name);
    SHOW(rknn_tensor_attr, n_elems);
    SHOW(rknn_tensor_attr, size);
    SHOW(rknn_tensor_attr, fmt);
    SHOW(rknn_tensor_attr, type);
    SHOW(rknn_tensor_attr, qnt_type);
    SHOW(rknn_tensor_attr, fl);
    SHOW(rknn_tensor_attr, zp);
    SHOW(rknn_tensor_attr, scale);
    SHOW(rknn_tensor_attr, pass_through);

    CHK(offsetof(rknn_tensor_attr, index)        == 0,   "index        offset == 0");
    CHK(offsetof(rknn_tensor_attr, n_dims)       == 4,   "n_dims       offset == 4");
    CHK(offsetof(rknn_tensor_attr, dims)         == 8,   "dims         offset == 8");
    CHK(offsetof(rknn_tensor_attr, name)         == 72,  "name         offset == 72");
    CHK(offsetof(rknn_tensor_attr, n_elems)      == 328, "n_elems      offset == 328");
    CHK(offsetof(rknn_tensor_attr, size)         == 332, "size         offset == 332");
    CHK(offsetof(rknn_tensor_attr, fmt)          == 336, "fmt          offset == 336");
    CHK(offsetof(rknn_tensor_attr, type)         == 340, "type         offset == 340");
    CHK(offsetof(rknn_tensor_attr, qnt_type)     == 344, "qnt_type     offset == 344");
    CHK(offsetof(rknn_tensor_attr, fl)           == 348, "fl           offset == 348");
    CHK(offsetof(rknn_tensor_attr, zp)           == 352, "zp           offset == 352");
    CHK(offsetof(rknn_tensor_attr, scale)        == 356, "scale        offset == 356");
    CHK(offsetof(rknn_tensor_attr, pass_through) == 368, "pass_through offset == 368");
    CHK(sizeof(rknn_tensor_attr) == 376,                 "sizeof == 376（runtime 1.6.0 实测要求）");

    printf("\n========================================================\n");
    if (g_fail == 0) printf(" 结果：全部通过（%d 项）\n", 0);
    else             printf(" 结果：%d 项不一致 ❌\n", g_fail);
    printf("========================================================\n");
    return g_fail == 0 ? 0 : 1;
}
