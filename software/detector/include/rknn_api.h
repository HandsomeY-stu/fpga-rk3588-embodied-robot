/*
 * rknn_api.h
 * 适配librknnrt 1.6.x的最小RKNN C API声明；配套ABI检查核对结构布局。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
#ifndef RKNN_API_H
#define RKNN_API_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RKNN_API_VERSION_MAJOR 1
#define RKNN_API_VERSION_MINOR 6

typedef enum {
    RKNN_SUCC = 0,
    RKNN_ERR_FAIL = -1,
    RKNN_ERR_TIMEOUT = -2,
    RKNN_ERR_DEVICE_UNAVAILABLE = -3,
    RKNN_ERR_MALLOC_FAIL = -4,
    RKNN_ERR_PARAM_INVALID = -5,
    RKNN_ERR_MODEL_INVALID = -6,
    RKNN_ERR_CTX_INVALID = -7,
    RKNN_ERR_INPUT_INVALID = -8,
    RKNN_ERR_OUTPUT_INVALID = -9,
    RKNN_ERR_DEVICE_UNMATCH = -10,
    RKNN_ERR_INCOMPATILE_PRE_COMPILE_MODEL = -11,
    RKNN_ERR_INCOMPATILE_OPTIMIZATION_LEVEL_VERSION = -12,
    RKNN_ERR_TARGET_PLATFORM_UNMATCH = -13,
} rknn_error_e;

typedef uint64_t rknn_context;

typedef enum {
    RKNN_TENSOR_FLOAT32 = 0,
    RKNN_TENSOR_FLOAT16 = 1,
    RKNN_TENSOR_INT8    = 2,
    RKNN_TENSOR_UINT8   = 3,
    RKNN_TENSOR_INT16   = 4,
    RKNN_TENSOR_UINT16  = 5,
    RKNN_TENSOR_INT32   = 6,
    RKNN_TENSOR_UINT32  = 7,
    RKNN_TENSOR_INT64   = 8,
    RKNN_TENSOR_BOOL    = 9,
    RKNN_TENSOR_INT4    = 10,
} rknn_tensor_type;

typedef enum {
    RKNN_TENSOR_NCHW = 0,
    RKNN_TENSOR_NHWC = 1,
    RKNN_TENSOR_NC1HWC2 = 2,
    RKNN_TENSOR_UNDEFINED = 3,
} rknn_tensor_format;

typedef enum {
    RKNN_TENSOR_QNT_NONE = 0,
    RKNN_TENSOR_QNT_DFP = 1,
    RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC = 2,
} rknn_tensor_qnt_type;

typedef struct {
    uint32_t index;
    uint32_t n_dims;
    uint32_t dims[16];
    char     name[256];
    uint32_t n_elems;
    uint32_t size;
    rknn_tensor_format   fmt;
    rknn_tensor_type     type;
    rknn_tensor_qnt_type qnt_type;
    int8_t   fl;
    int8_t   _pad0[3];
    int32_t  zp;
    float    scale;
    uint32_t _pad1;         
    uint32_t _pad2;         
    uint8_t  pass_through;
    uint8_t  _pad3[3];
    uint32_t _pad4;         
} rknn_tensor_attr;

#ifdef __cplusplus
static_assert(sizeof(rknn_tensor_attr) == 376,
              "rknn_tensor_attr 必须是 376 字节（runtime 1.6.0 实测要求）");
#else
_Static_assert(sizeof(rknn_tensor_attr) == 376,
               "rknn_tensor_attr 必须是 376 字节");
#endif

typedef struct {
    uint32_t          index;
    uint32_t          _pad0;
    void             *buf;
    uint32_t          size;
    uint8_t           pass_through;
    uint8_t           _pad1[3];
    rknn_tensor_type  type;
    rknn_tensor_format fmt;
} rknn_input;

typedef struct {
    uint8_t           want_float;     
    uint8_t           is_prealloc;    

    uint32_t          index;          
    void             *buf;            
    uint32_t          size;           
} rknn_output;

#ifdef __cplusplus
static_assert(sizeof(rknn_output) == 24, "rknn_output 必须是 24 字节");
static_assert(offsetof(rknn_output, want_float) == 0,  "want_float offset 必须为 0");
static_assert(offsetof(rknn_output, is_prealloc) == 1, "is_prealloc offset 必须为 1");
static_assert(offsetof(rknn_output, index) == 4,       "index offset 必须为 4");
static_assert(offsetof(rknn_output, buf) == 8,         "buf offset 必须为 8");
static_assert(offsetof(rknn_output, size) == 16,       "size offset 必须为 16");
#else
_Static_assert(sizeof(rknn_output) == 24, "rknn_output 必须是 24 字节");
_Static_assert(offsetof(rknn_output, index) == 4, "index offset 必须为 4");
#endif

typedef struct {
    uint32_t         flags;       
    void            *privileged_mem;   
} rknn_init_extend;

typedef enum {
    RKNN_QUERY_IN_OUT_NUM = 0,
    RKNN_QUERY_INPUT_ATTR = 1,
    RKNN_QUERY_OUTPUT_ATTR = 2,
    RKNN_QUERY_PERF_DETAIL = 3,
    RKNN_QUERY_PERF_RUN = 4,
    RKNN_QUERY_SDK_VERSION = 5,
    RKNN_QUERY_MEM_SIZE = 6,
    RKNN_QUERY_CUSTOM_STRING = 7,
    RKNN_QUERY_NATIVE_INPUT_ATTR = 8,
    RKNN_QUERY_NATIVE_OUTPUT_ATTR = 9,
    RKNN_QUERY_NATIVE_NC1HWC2_INPUT_ATTR = 8,
    RKNN_QUERY_NATIVE_NC1HWC2_OUTPUT_ATTR = 9,
    RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR = 10,
    RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR = 11,
    RKNN_QUERY_DEVICE_MEM_INFO = 12,
    RKNN_QUERY_INPUT_DYNAMIC_RANGE = 13,
    RKNN_QUERY_CURRENT_INPUT_ATTR = 14,
    RKNN_QUERY_CURRENT_OUTPUT_ATTR = 15,
    RKNN_QUERY_CURRENT_NATIVE_INPUT_ATTR = 16,
    RKNN_QUERY_CURRENT_NATIVE_OUTPUT_ATTR = 17,
} rknn_query_cmd;

typedef struct {
    uint32_t n_input;
    uint32_t n_output;
} rknn_input_output_num;

#define RKNN_MAX_VERSION_LEN 256
typedef struct {
    char api_version[RKNN_MAX_VERSION_LEN];
    char drv_version[RKNN_MAX_VERSION_LEN];
} rknn_sdk_version;

#define RKNN_FLAG_COLLECT_PERF_MASK     (1 << 0)
#define RKNN_FLAG_MEM_ALLOC_OUTSIDE     (1 << 1)

int rknn_init(rknn_context *context, void *model, uint32_t size,
              uint32_t flag, rknn_init_extend *extend);

int rknn_query(rknn_context context, rknn_query_cmd cmd,
               void *info, uint32_t size);

int rknn_inputs_set(rknn_context context, uint32_t n_inputs, rknn_input inputs[]);

int rknn_run(rknn_context context, void *extend);

int rknn_wait(rknn_context context, void *extend);

int rknn_outputs_get(rknn_context context, uint32_t n_outputs,
                     rknn_output outputs[], void *extend);

int rknn_outputs_release(rknn_context context, uint32_t n_outputs,
                         rknn_output outputs[]);

int rknn_destroy(rknn_context context);

#ifdef __cplusplus
}
#endif

#endif  
