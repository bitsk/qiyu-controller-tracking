// Optional ARM32 SNPE backend selection; the stock SDK and models stay intact.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// libSNPE.so a4415e78: RuntimeList C1 @0x316988 stores one PIMPL pointer;
// D1 @0x316bd4 owns it. setRuntimeProcessorOrder @0x7582e copies the list.
// Its Runtime_t table @0x4c3fd0 names 2=dsp_fixed8_tf, 5=aip_fixed8_tf.
struct runtime_list { void *impl; };
_Static_assert(sizeof(struct runtime_list) == 4, "This shim uses the stock ARM32 ABI");

#define SET_ORDER "_ZN3zdl4SNPE11SNPEBuilder24setRuntimeProcessorOrderERKNS_8DlSystem11RuntimeListE"
#define AVAILABLE "_ZN3zdl4SNPE11SNPEFactory18isRuntimeAvailableENS_8DlSystem9Runtime_tE"
#define LAST_ERROR_CODE "_ZN3zdl8DlSystem16getLastErrorCodeEv"
#define LAST_ERROR_STRING "_ZN3zdl8DlSystem18getLastErrorStringEv"
#define LIST_CTOR "_ZN3zdl8DlSystem11RuntimeListC1Ev"
#define LIST_DTOR "_ZN3zdl8DlSystem11RuntimeListD1Ev"
#define LIST_ADD "_ZN3zdl8DlSystem11RuntimeList3addERKNS0_9Runtime_tE"
#define LIST_SIZE "_ZNK3zdl8DlSystem11RuntimeList4sizeEv"
#define LIST_AT "_ZN3zdl8DlSystem11RuntimeListixEj"

static pthread_once_t api_once = PTHREAD_ONCE_INIT;
static void *snpe;
static void *(*set_order)(void *, const struct runtime_list *);
static bool (*available)(int);
static int (*last_error_code)(void);
static const char *(*last_error_string)(void);
static void (*list_ctor)(struct runtime_list *);
static void (*list_dtor)(struct runtime_list *);
static bool (*list_add)(struct runtime_list *, const int *);
static unsigned (*list_size)(const struct runtime_list *);
static const int *(*list_at)(struct runtime_list *, unsigned);

static void *symbol(const char *name)
{
    void *result = dlsym(snpe, name);
    if (!result) {
        fprintf(stderr, "A081 SNPE runtime shim: missing %s: %s\n", name, dlerror());
        _exit(78);
    }
    return result;
}

static void load_api(void)
{
    // The SDK loads RTLD_LOCAL. An explicit handle finds the original symbols
    // in that object without relying on RTLD_NEXT's global lookup scope.
    snpe = dlopen("libSNPE.so", RTLD_NOW | RTLD_LOCAL);
    if (!snpe) {
        fprintf(stderr, "A081 SNPE runtime shim: libSNPE.so: %s\n", dlerror());
        _exit(78);
    }
    set_order = symbol(SET_ORDER);
    available = symbol(AVAILABLE);
    last_error_code = symbol(LAST_ERROR_CODE);
    last_error_string = symbol(LAST_ERROR_STRING);
    list_ctor = symbol(LIST_CTOR);
    list_dtor = symbol(LIST_DTOR);
    list_add = symbol(LIST_ADD);
    list_size = symbol(LIST_SIZE);
    list_at = symbol(LIST_AT);
}

static void quote_error(const char *text)
{
    fputc('"', stderr);
    for (const unsigned char *p = (const unsigned char *)(text ? text : ""); *p; ++p) {
        if (*p == '"' || *p == '\\') {
            fputc('\\', stderr);
            fputc(*p, stderr);
        } else if (*p < 0x20) {
            fprintf(stderr, "\\u%04x", *p);
        } else {
            fputc(*p, stderr);
        }
    }
    fputc('"', stderr);
}

__attribute__((visibility("default")))
void *a081_snpe_set_order(void *builder, const struct runtime_list *original)
    __asm__(SET_ORDER);

void *a081_snpe_set_order(void *builder, const struct runtime_list *original)
{
    pthread_once(&api_once, load_api);
    const char *selection = getenv("A081_SNPE_RUNTIME");
    if (!selection || strcmp(selection, "dsp") != 0)
        return set_order(builder, original);

    const unsigned count = list_size(original);
    const int first = count ? *list_at((struct runtime_list *)original, 0) : -1;
    // Never interpose isRuntimeAvailable: AIP's original failure and fallback
    // remain truthful. Only a successful, real DSP probe changes this order.
    const bool dsp_available = available(2);
    if (!dsp_available) {
        const int error_code = last_error_code();
        const char *error_string = last_error_string();
        fprintf(stderr, "{\"kind\":\"controller_snpe_runtime\",\"requested\":\"dsp\","
                "\"dsp_available\":false,\"override_applied\":false,"
                "\"original_count\":%u,\"original_first\":%d,"
                "\"last_error_code\":%d,\"last_error_string\":", count, first, error_code);
        quote_error(error_string);
        fputs("}\n", stderr);
        return set_order(builder, original);
    }

    struct runtime_list dsp_order;
    const int dsp = 2;
    list_ctor(&dsp_order);
    const bool added = list_add(&dsp_order, &dsp);
    void *result = set_order(builder, added ? &dsp_order : original);
    list_dtor(&dsp_order);
    fprintf(stderr, "{\"kind\":\"controller_snpe_runtime\",\"requested\":\"dsp\","
            "\"dsp_available\":true,\"override_applied\":%s,"
            "\"original_count\":%u,\"original_first\":%d,\"selected_first\":%d}\n",
            added ? "true" : "false", count, first, added ? dsp : first);
    return result;
}
