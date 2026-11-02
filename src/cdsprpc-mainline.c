/* SPDX-License-Identifier: GPL-3.0-or-later
 * ARM32/Bionic Qualcomm client ABI over the existing mainline HexagonRPC path.
 * This owns one CDSP user PD. The head tracker's separate root-PD file service
 * must already be running. No Android vendor ioctl or ION ABI is emulated.
 * OEM API shapes: local libcdsprpc 07392760, oem-*.json in this directory.
 */
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <misc/fastrpc.h>
#include <libhexagonrpc/error.h>
#include <libhexagonrpc/handle.h>
#include <libhexagonrpc/interface/remotectl.h>
#include "apps_mem.h"
#include "apps_std.h"
#include "interface/adsp_default_listener.h"
#include "listener.h"
#include "localctl.h"
#include "rpcd_builder.h"

/* ARM32 remote_arg is an eight-byte union, including the DMA fd/offset form. */
typedef union {
    struct { void *pv; size_t nLen; } buf;
    uint32_t h;
    uint64_t h64;
    struct { int fd; uint32_t offset; } dma;
} remote_arg;
_Static_assert(sizeof(void *) == 4 && sizeof(remote_arg) == 8, "ARM32 ABI");
_Static_assert(sizeof(struct fastrpc_invoke_args) == 24, "mainline invoke ABI");

static pthread_mutex_t session_lock = PTHREAD_MUTEX_INITIALIZER;
static int session_fd = -1;
static int session_result = -1;
static bool unsigned_pd;
static struct fastrpc_interface *interfaces[3];
static _Atomic unsigned event_counts[12];
static _Atomic unsigned scalar_trace_count;
#define HANDLE_PREFIX UINT64_C(0xa081000300000000)

static void event(unsigned kind, const char *name, uint64_t arg, int rc)
{
    if (kind < 12 && atomic_fetch_add(&event_counts[kind], 1) < 16)
        fprintf(stderr, "a081_cdsp stage=%s arg=0x%" PRIx64 " rc=%d errno=%d\n",
                name, arg, rc, rc < 0 ? errno : 0);
}

static int result(int rc) { return rc < 0 ? AEE_EFAILED : rc; }

static int create_pd(int fd, const char *shell, uint32_t attrs)
{
    int source = open(shell, O_RDONLY | O_CLOEXEC), rc = -1, saved;
    struct stat st;
    struct fastrpc_alloc_dma_buf dma = { .fd = -1 };
    struct fastrpc_init_create init = {.attrs=attrs};
    void *mapped = MAP_FAILED;
    if (source < 0) return -1;
    if (fstat(source, &st) || st.st_size <= 0 || st.st_size > INT_MAX) goto done;
    dma.size = ((uint64_t)st.st_size + 4095u) & ~UINT64_C(4095);
    if (ioctl(fd, FASTRPC_IOCTL_ALLOC_DMA_BUFF, &dma)) goto done;
    mapped = mmap(NULL, (size_t)dma.size, PROT_READ | PROT_WRITE, MAP_SHARED, dma.fd, 0);
    if (mapped == MAP_FAILED) goto done;
    size_t offset = 0;
    while (offset < (size_t)st.st_size) {
        ssize_t count = read(source, (char *)mapped + offset, (size_t)st.st_size - offset);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) goto done;
        offset += (size_t)count;
    }
    init.file = (uintptr_t)mapped;
    init.filefd = dma.fd;
    init.filelen = (uint32_t)st.st_size;
    rc = ioctl(fd, FASTRPC_IOCTL_INIT_CREATE, &init);
done:
    saved = errno;
    if (mapped != MAP_FAILED) munmap(mapped, (size_t)dma.size);
    if (dma.fd >= 0) close(dma.fd);
    close(source);
    errno = saved;
    return rc;
}

static void *listener_main(void *unused)
{
    (void)unused;
    pthread_setname_np(pthread_self(), "a081-cdsp-rx");
    event(0, "listener_ready", (uint64_t)session_fd, 0);
    int rc = run_fastrpc_listener(session_fd, 3, interfaces);
    event(0, "listener_exit", (uint64_t)session_fd, rc);
    return NULL;
}

/* A capability query is legal before creating a PD. In particular SNPE v2
 * queries attribute1 and then requests an unsigned PD before its first open. */
static int open_device_locked(void)
{
    if (session_fd >= 0) return 0;
    const char *device = getenv("A081_CDSP_DEVICE");
    if (!device) device = "/dev/fastrpc-cdsp";
    session_fd = open(device, O_RDWR | O_CLOEXEC);
    event(0, "device_open", 3, session_fd < 0 ? -1 : 0);
    return session_fd < 0 ? AEE_EFAILED : 0;
}

static int ensure_device(void)
{
    pthread_mutex_lock(&session_lock);
    int rc = open_device_locked();
    pthread_mutex_unlock(&session_lock);
    return rc;
}

static int shell_path(char *path, size_t capacity, bool use_unsigned)
{
    const char *root = getenv("A081_CDSP_RUNTIME");
    if (!root || root[0] != '/') return AEE_EBADPARM;
    const char *name = use_unsigned ? "fastrpc_shell_unsigned_3" : "fastrpc_shell_3";
    if (snprintf(path, capacity, "%s/dsp/cdsp/%s", root, name) >= (int)capacity)
        return AEE_EBADPARM;
    return 0;
}

static int ensure_session(void)
{
    pthread_mutex_lock(&session_lock);
    if (session_result != -1) goto done;
    session_result = AEE_EFAILED;
    const char *root = getenv("A081_CDSP_RUNTIME");
    char shell[PATH_MAX], error[256] = {0}, fdtext[24];
    uint32_t listener_handle;
    pthread_t thread;
    if (!root || root[0] != '/') { event(0, "missing_runtime", 0, AEE_EBADPARM); goto done; }
    if (shell_path(shell, sizeof(shell), unsigned_pd) || open_device_locked()) goto done;
    uint32_t attrs = unsigned_pd ? FASTRPC_MODE_UNSIGNED_MODULE : 0;
    int rc = create_pd(session_fd, shell, attrs);
    event(0, "init_create", attrs, rc);
    if (rc) goto failed;
    snprintf(fdtext, sizeof(fdtext), "%d", session_fd);
    setenv("HEXAGONRPC_FD", fdtext, 1);
    struct hexagonfs_dirent *root_dir = construct_root_dir(root, "cdsp");
    if (!root_dir) goto failed;
    interfaces[0] = fastrpc_localctl_init(3, interfaces);
    interfaces[1] = fastrpc_apps_std_init(root_dir);
    interfaces[2] = fastrpc_apps_mem_init(session_fd);
    if (!interfaces[0] || !interfaces[1] || !interfaces[2]) goto failed;
    rc = hexagonrpc_open(session_fd, "adsp_default_listener", &listener_handle, sizeof(error), error);
    if (rc) { fprintf(stderr, "a081_cdsp listener_open_error=%s\n", error); goto failed; }
    rc = adsp_default_listener_register(session_fd, listener_handle);
    hexagonrpc_close(session_fd, listener_handle);
    event(0, "listener_register", 0, rc);
    if (rc) goto failed;
    rc = pthread_create(&thread, NULL, listener_main, NULL);
    if (rc) { errno = rc; goto failed; }
    pthread_detach(thread);
    session_result = 0;
    goto done;
failed:
    close(session_fd);
    session_fd = -1;
done:
    ;
    int rc_out = session_result;
    pthread_mutex_unlock(&session_lock);
    return rc_out;
}

struct registration {
    struct registration *next;
    void *pointer;
    size_t size, mapped_size;
    int fd;
    unsigned attr;
    bool owned_fd, owned_mapping;
};
static pthread_mutex_t memory_lock = PTHREAD_MUTEX_INITIALIZER;
static struct registration *registrations;

static struct registration *find_pointer(const void *pointer, size_t size)
{
    uintptr_t p = (uintptr_t)pointer;
    for (struct registration *r = registrations; r; r = r->next) {
        uintptr_t base = (uintptr_t)r->pointer;
        if (r->pointer && p >= base && p - base <= r->size && size <= r->size - (p - base))
            return r;
    }
    return NULL;
}

static struct registration *find_fd(int fd)
{
    for (struct registration *r = registrations; r; r = r->next)
        if (r->fd == fd) return r;
    return NULL;
}

void rpcmem_init(void) {}
void rpcmem_deinit(void) {} /* other live SDK instances share this process */

void *rpcmem_alloc(int heap, uint32_t flags, int size)
{
    if (size <= 0 || ensure_session()) return NULL;
    struct fastrpc_alloc_dma_buf dma = { .fd = -1,
        .size = ((uint64_t)(uint32_t)size + 4095u) & ~UINT64_C(4095) };
    /* The mainline allocator exports dma_alloc_coherent memory. Heap/default
     * cache preferences are observed, never passed as incompatible ION flags. */
    event(1, "rpcmem_alloc", ((uint64_t)(uint32_t)heap << 32) | flags, size);
    if (ioctl(session_fd, FASTRPC_IOCTL_ALLOC_DMA_BUFF, &dma)) return NULL;
    void *p = mmap(NULL, (size_t)dma.size, PROT_READ | PROT_WRITE, MAP_SHARED, dma.fd, 0);
    if (p == MAP_FAILED) { close(dma.fd); return NULL; }
    struct registration *r = calloc(1, sizeof(*r));
    if (!r) { munmap(p, (size_t)dma.size); close(dma.fd); return NULL; }
    *r = (struct registration){ .pointer=p, .size=(size_t)size, .mapped_size=(size_t)dma.size,
        .fd=dma.fd, .owned_fd=true, .owned_mapping=true };
    pthread_mutex_lock(&memory_lock);
    r->next = registrations; registrations = r;
    pthread_mutex_unlock(&memory_lock);
    return p;
}

void rpcmem_free(void *p)
{
    pthread_mutex_lock(&memory_lock);
    struct registration **link = &registrations, *r;
    while (*link && !((*link)->pointer == p && (*link)->owned_fd)) link = &(*link)->next;
    r = *link;
    if (r) *link = r->next;
    pthread_mutex_unlock(&memory_lock);
    if (!r) return;
    if (r->owned_mapping) munmap(r->pointer, r->mapped_size);
    close(r->fd);
    free(r);
}

int rpcmem_to_fd(void *p)
{
    pthread_mutex_lock(&memory_lock);
    struct registration *r = find_pointer(p, 0);
    int fd = r ? r->fd : -1;
    pthread_mutex_unlock(&memory_lock);
    return fd;
}
void *rpcmem_alloc_internal(int h, uint32_t f, int n) { return rpcmem_alloc(h, f, n); }
void rpcmem_free_internal(void *p) { rpcmem_free(p); }
int rpcmem_to_fd_internal(void *p) { return rpcmem_to_fd(p); }

void remote_register_buf_attr(void *p, int size, int fd, unsigned attr)
{
    if (!p || size <= 0) return;
    pthread_mutex_lock(&memory_lock);
    struct registration **link = &registrations;
    while (*link && (*link)->pointer != p) link = &(*link)->next;
    struct registration *r = *link;
    if (fd < 0) {
        if (r && !r->owned_fd) {
            *link = r->next;
            if (r->owned_mapping) munmap(r->pointer, r->mapped_size);
            free(r);
        }
    } else if (r) {
        if (!r->owned_fd) { r->fd=fd; r->size=(size_t)size; r->attr=attr; }
    } else {
        r = calloc(1, sizeof(*r));
        if (r) {
            *r = (struct registration){ .pointer=p, .size=(size_t)size, .fd=fd, .attr=attr,
                                      .next=registrations };
            registrations = r;
        }
    }
    pthread_mutex_unlock(&memory_lock);
    event(2, "register_buf", attr, fd);
}
void remote_register_buf(void *p, int size, int fd) { remote_register_buf_attr(p, size, fd, 0); }

int remote_register_dma_handle_attr(int fd, uint32_t size, unsigned attr)
{
    if (fd < 0 || !size || attr != 0) return attr ? AEE_EUNSUPPORTED : AEE_EBADPARM;
    pthread_mutex_lock(&memory_lock);
    struct registration *r = find_fd(fd);
    if (!r) {
        r = calloc(1, sizeof(*r));
        if (r) { *r=(struct registration){.fd=fd, .size=size, .next=registrations}; registrations=r; }
    }
    pthread_mutex_unlock(&memory_lock);
    event(2, "register_dma", size, fd);
    return r ? 0 : AEE_ENOMEMORY;
}
int remote_register_dma_handle(int fd, uint32_t size) { return remote_register_dma_handle_attr(fd,size,0); }

void *remote_register_fd(int fd, int size)
{
    /* OEM reserves a PROT_NONE anonymous range. Mainline derives sub-buffer
     * offsets from the DMA-buf VMA, so reserve the same inaccessible range by
     * mapping the actual fd. It is a registration token, not CPU-readable. */
    if (fd < 0 || size <= 0) return NULL;
    void *p = mmap(NULL, (size_t)size, PROT_NONE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) return NULL;
    struct registration *r = calloc(1, sizeof(*r));
    if (!r) { munmap(p, (size_t)size); return NULL; }
    *r=(struct registration){.pointer=p, .size=(size_t)size, .mapped_size=(size_t)size,
                            .fd=fd, .owned_mapping=true};
    pthread_mutex_lock(&memory_lock);
    r->next=registrations; registrations=r;
    pthread_mutex_unlock(&memory_lock);
    return p;
}

int remote_handle64_open(const char *name, uint64_t *handle)
{
    if (!name || !handle) return AEE_EBADPARM;
    if (strstr(name, "_dom=") && !strstr(name, "_dom=cdsp")) return AEE_EUNSUPPORTED;
    int rc=ensure_session();
    if (rc) return rc;
    char *uri=strdup(name), error[512]={0};
    uint32_t raw=0;
    if (!uri) return AEE_ENOMEMORY;
    char *domain=strstr(uri,"&_dom=cdsp");
    if (domain) memmove(domain, domain+10, strlen(domain+10)+1);
    rc=hexagonrpc_open(session_fd,uri,&raw,sizeof(error),error);
    fprintf(stderr,"a081_cdsp stage=open uri=%s raw_handle=0x%x rc=%d detail=%s\n",uri,raw,rc,error);
    free(uri);
    if (!rc) *handle=HANDLE_PREFIX | raw;
    return result(rc);
}

int remote_handle64_close(uint64_t handle)
{
    if ((handle & UINT64_C(0xffffffff00000000)) != HANDLE_PREFIX || ensure_session())
        return AEE_EBADHANDLE;
    char error[256]={0}; uint32_t remote_error=0;
    int rc=remotectl_close(session_fd,(uint32_t)handle,sizeof(error),error,&remote_error);
    event(3,"close",handle,rc);
    return rc ? result(rc) : remote_error ? AEE_EUNABLETOUNLOAD : 0;
}

/* Explicit, bounded observation of the SNPE availability exchange. Buffer
 * contents and return values remain untouched. Normal runs do not log data. */
static unsigned trace_scalar_call(uint32_t scalar)
{
    const char *value = getenv("A081_CDSP_TRACE_SCALAR");
    if (scalar != UINT32_C(0x08020200) || !value || strcmp(value,"0x08020200")) return 0;
    unsigned number = atomic_fetch_add(&scalar_trace_count,1);
    return number < 4 ? number + 1 : 0;
}

static void trace_buffers(unsigned call, const char *stage, unsigned inputs,
                          unsigned start, unsigned end, const struct fastrpc_invoke_args *wire)
{
    for (unsigned i=start; i<end; ++i) {
        char hex[193]; size_t count=0;
        /* Registration-only DMA tokens can be PROT_NONE. This method normally
         * uses ordinary stack buffers; never dereference a registered token. */
        if (wire[i].ptr && wire[i].fd < 0) {
            count=wire[i].length < 96 ? (size_t)wire[i].length : 96;
            const unsigned char *bytes=(const unsigned char *)(uintptr_t)wire[i].ptr;
            for (size_t j=0; j<count; ++j) {
                static const char digits[]="0123456789abcdef";
                hex[j*2]=digits[bytes[j]>>4]; hex[j*2+1]=digits[bytes[j]&15];
            }
        }
        hex[count*2]='\0';
        fprintf(stderr,"a081_cdsp_trace scalar=0x08020200 call=%u stage=%s direction=%s"
                " arg=%u nLen=%" PRIu64 " fd=%d captured=%zu hex=%s\n",
                call,stage,i<inputs ? "in" : "out",i,(uint64_t)wire[i].length,wire[i].fd,count,hex);
    }
}

int remote_handle64_invoke(uint64_t handle, uint32_t scalar, remote_arg *args)
{
    unsigned buffers=((scalar>>16)&255u)+((scalar>>8)&255u);
    unsigned count=buffers+((scalar>>4)&15u)+(scalar&15u);
    if ((handle & UINT64_C(0xffffffff00000000)) != HANDLE_PREFIX) return AEE_EBADHANDLE;
    if (count && !args) return AEE_EBADPARM;
    if (ensure_session()) return AEE_EFAILED;
    struct fastrpc_invoke_args *wire=calloc(count ? count : 1,sizeof(*wire));
    if (!wire) return AEE_ENOMEMORY;
    int rc=0;
    pthread_mutex_lock(&memory_lock);
    for (unsigned i=0; i<count; ++i) {
        wire[i].fd=-1;
        if (i < buffers) {
            wire[i].ptr=(uintptr_t)args[i].buf.pv;
            wire[i].length=args[i].buf.nLen;
            struct registration *r=find_pointer(args[i].buf.pv,args[i].buf.nLen);
            if (r) {
                if (r->attr) { rc=AEE_EUNSUPPORTED; break; }
                wire[i].fd=r->fd;
            }
        } else {
            struct registration *r=find_fd(args[i].dma.fd);
            if (!r || r->attr || args[i].dma.offset > r->size) { rc=AEE_EBADPARM; break; }
            wire[i].fd=r->fd;
            wire[i].ptr=args[i].dma.offset;
            wire[i].length=r->size;
        }
    }
    pthread_mutex_unlock(&memory_lock);
    if (!rc) {
        struct fastrpc_invoke inv={.handle=(uint32_t)handle,.sc=scalar,.args=(uintptr_t)wire};
        unsigned trace=trace_scalar_call(scalar), inputs=(scalar>>16)&255u;
        if (trace) trace_buffers(trace,"before",inputs,0,buffers,wire);
        rc=ioctl(session_fd,FASTRPC_IOCTL_INVOKE,&inv);
        if (trace) {
            int saved=errno;
            trace_buffers(trace,"after",inputs,inputs,buffers,wire);
            errno=saved;
        }
    }
    event(rc ? 5 : 4,"invoke",scalar,rc);
    free(wire);
    return result(rc);
}

int remote_handle_open(const char *name, uint32_t *handle)
{
    uint64_t h=0; if (!handle) return AEE_EBADPARM;
    int rc=remote_handle64_open(name,&h); if (!rc) *handle=(uint32_t)h; return rc;
}
int remote_handle_close(uint32_t h) { return remote_handle64_close(HANDLE_PREFIX|h); }
int remote_handle_invoke(uint32_t h,uint32_t sc,remote_arg *a) { return remote_handle64_invoke(HANDLE_PREFIX|h,sc,a); }

static int control(uint32_t request, void *data, uint32_t length)
{
    if (request==4 && data && length==4) { *(uint32_t *)data=3; return 0; }
    if (request==2 && data && length==12) {
        uint32_t *words=data;
        if (words[0]!=3) return AEE_EUNSUPPORTED;
        int rc=ensure_device(); if (rc) return rc;
        struct fastrpc_ioctl_capability cap={.attribute_id=words[1]};
        rc=ioctl(session_fd,FASTRPC_IOCTL_GET_DSP_INFO,&cap);
        if (!rc) words[2]=cap.capability;
        event(6,"capability",((uint64_t)words[1]<<32)|cap.capability,rc);
        return result(rc);
    }
    event(7,"unsupported_control",((uint64_t)request<<32)|length,AEE_EUNSUPPORTED);
    return AEE_EUNSUPPORTED;
}
int remote_handle_control(uint32_t request,void *data,uint32_t length) { return control(request,data,length); }
int remote_handle64_control(uint64_t h,uint32_t request,void *data,uint32_t length)
{
    if ((h & UINT64_C(0xffffffff00000000)) != HANDLE_PREFIX) return AEE_EBADHANDLE;
    return control(request,data,length);
}
int remote_session_control(uint32_t request,void *data,uint32_t length)
{
    if (request == 2 && data && length == 8) {
        uint32_t words[2]; memcpy(words,data,sizeof(words));
        if (words[0] != 3 || words[1] > 1) return AEE_EBADPARM;
        bool requested = words[1] != 0;
        char shell[PATH_MAX];
        pthread_mutex_lock(&session_lock);
        int rc;
        if (session_result == 0) {
            rc = unsigned_pd == requested ? 0 : AEE_ENOTALLOWED;
        } else if (session_result != -1) {
            rc = AEE_EBADSTATE;
        } else {
            rc = shell_path(shell,sizeof(shell),requested);
            if (!rc && access(shell,R_OK)) rc = AEE_ERESOURCENOTFOUND;
            if (!rc) unsigned_pd = requested;
        }
        pthread_mutex_unlock(&session_lock);
        event(8,"unsigned_pd_configuration",requested,rc);
        return rc;
    }
    /* DSP thread/QoS policies have no translated contract. */
    uint32_t first=0; if (data && length>=4) memcpy(&first,data,4);
    event(8,"unsupported_session_control",((uint64_t)request<<32)|first,AEE_EUNSUPPORTED);
    return AEE_EUNSUPPORTED;
}

struct mapping { struct mapping *next; uint64_t address,size; int fd; };
static struct mapping *mappings;
int remote_mmap64(int fd,uint32_t flags,uint64_t input,int64_t size,uint64_t *output)
{
    if (fd<0 || flags || size<=0 || !output || input>UINT32_MAX) return AEE_EBADPARM;
    if (ensure_session()) return AEE_EFAILED;
    struct mapping *map=calloc(1,sizeof(*map)); if (!map) return AEE_ENOMEMORY;
    struct fastrpc_mem_map req={.fd=fd,.flags=FASTRPC_MAP_STATIC,.vaddrin=input,.length=(uint64_t)size};
    int rc=ioctl(session_fd,FASTRPC_IOCTL_MEM_MAP,&req);
    event(9,"mmap64",(uint64_t)size,rc);
    if (rc) { free(map); return result(rc); }
    *output=req.vaddrout;
    *map=(struct mapping){.address=req.vaddrout,.size=(uint64_t)size,.fd=fd};
    pthread_mutex_lock(&memory_lock); map->next=mappings; mappings=map; pthread_mutex_unlock(&memory_lock);
    return 0;
}
int remote_munmap64(uint64_t address,int64_t size)
{
    pthread_mutex_lock(&memory_lock);
    struct mapping **link=&mappings;
    while (*link && ((*link)->address!=address || (*link)->size!=(uint64_t)size)) link=&(*link)->next;
    struct mapping *map=*link;
    if (!map) { pthread_mutex_unlock(&memory_lock); return AEE_EBADPARM; }
    struct fastrpc_mem_unmap req={.fd=map->fd,.vaddr=address,.length=(uint64_t)size};
    int rc=ioctl(session_fd,FASTRPC_IOCTL_MEM_UNMAP,&req);
    if (!rc) { *link=map->next; free(map); }
    pthread_mutex_unlock(&memory_lock);
    event(9,"munmap64",address,rc);
    return result(rc);
}
