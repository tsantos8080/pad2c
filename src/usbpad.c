#include "usbpad.h"
#include "log.h"
#include "util.h"

#include <sys/ioctl.h>
#include <sys/types.h>

#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define VID_8BITDO  0x2DC8
#define N_RD        4           /* reads kept pending on the IN endpoint */
#define RD_MAX      64
#define T_ALIVE     1000        /* ms between checks that the device is still there */

struct usbpad {
    int fd;
    char path[24];
    char name[64];
    uint8_t ep_in;
    uint16_t mps;
    int xinput, nreads, enable_pending;
    uint16_t pid;
    uint8_t ep_out;
    unsigned char enable[3];
    long t_enable;
    struct usb_fs_endpoint ep[N_RD + 1];
    void *bufp[N_RD + 1][1];
    uint32_t lenp[N_RD + 1][1];
    unsigned char buf[N_RD][RD_MAX];
    unsigned char busy[N_RD];
    long t_alive;
    int logged_first;
};

static int dead_errno(int e)
{
    return e == ENXIO || e == ENODEV || e == EIO || e == EBADF || e == ENOENT || e == ENOTTY;
}

/* The first interrupt IN endpoint of a HID interface. */
static int find_in_endpoint(int fd, uint8_t *ep, uint16_t *mps, int xinput, uint8_t *out)
{
    struct usb_gen_descriptor gd;
    unsigned char cfg[1024];
    int off = 0, len, hid = 0, found = 0;

    memset(&gd, 0, sizeof gd);
    gd.ugd_data = cfg;
    gd.ugd_maxlen = sizeof cfg;
    gd.ugd_config_index = 0xFF;
    if (ioctl(fd, USB_GET_FULL_DESC, &gd) != 0) return 0;
    len = gd.ugd_actlen < (int)sizeof cfg ? gd.ugd_actlen : (int)sizeof cfg;
    while (off + 2 <= len) {
        int dl = cfg[off];
        const unsigned char *d = cfg + off;
        if (dl < 2 || off + dl > len) break;
        if (d[1] == 0x04 && dl >= 9)
            hid = d[3] == 0 && (xinput
                ? d[2] == 0 && d[5] == 0xff && d[6] == 0x5d && d[7] == 1
                : d[5] == 0x03);
        else if (d[1] == 0x05 && dl >= 7 && hid && (d[2] & 0x80) && (d[3] & 3) == 3) {
            if (xinput && (d[2] != 0x84 || d[4] != 32 || d[5] != 0)) return 0;
            *ep = d[2];
            *mps = (uint16_t)(d[4] | d[5] << 8);
            if (!xinput) return 1;
            found = 1;
        } else if (xinput && hid && d[1] == 0x05 && dl >= 7 &&
                   d[2] == 0x05 && (d[3] & 3) == 3 && d[4] == 32 && d[5] == 0) {
            *out = d[2];
        }
        off += dl;
    }
    return found && *out == 0x05;
}

static int start_read(usbpad *u, int i)
{
    struct usb_fs_start st;

    u->lenp[i][0] = u->mps;
    u->ep[i].nFrames = 1;
    u->ep[i].aFrames = 0;
    u->ep[i].status = 0;
    memset(&st, 0, sizeof st);
    st.ep_index = (uint8_t)i;
    if (ioctl(u->fd, USB_FS_START, &st) != 0) return dead_errno(errno) ? -1 : 0;
    u->busy[i] = 1;
    return 1;
}

static usbpad *try_node(int bus, int addr)
{
    struct usb_device_descriptor dd;
    struct usb_device_info di;
    struct usb_fs_init init;
    usbpad *u;
    int i;

    u = calloc(1, sizeof *u);
    if (!u) return NULL;
    snprintf(u->path, sizeof u->path, "/dev/ugen%d.%d", bus, addr);
    u->fd = open(u->path, O_RDWR);
    if (u->fd < 0) goto fail;
    if (ioctl(u->fd, USB_GET_DEVICE_DESC, &dd) != 0) goto fail;
    if (UGETW(dd.idVendor) != VID_8BITDO) goto fail;
    u->pid = UGETW(dd.idProduct);
    if (u->pid == 0x301c) goto fail; /* idle dongle */
    u->xinput = u->pid == 0x310a;
    u->nreads = u->xinput ? 1 : N_RD;
    if (!find_in_endpoint(u->fd, &u->ep_in, &u->mps, u->xinput, &u->ep_out)) goto fail;
    if (u->mps == 0 || u->mps > RD_MAX) u->mps = RD_MAX;

    memset(&di, 0, sizeof di);
    if (ioctl(u->fd, USB_GET_DEVICEINFO, &di) == 0 && di.udi_product[0])
        snprintf(u->name, sizeof u->name, "%.60s", di.udi_product);
    else
        snprintf(u->name, sizeof u->name, "8BitDo %04x", UGETW(dd.idProduct));

    for (i = 0; i < u->nreads; i++) {
        u->bufp[i][0] = u->buf[i];
        u->lenp[i][0] = u->mps;
        u->ep[i].ppBuffer = u->bufp[i];
        u->ep[i].pLength = u->lenp[i];
        u->ep[i].nFrames = 1;
        u->ep[i].flags = USB_FS_FLAG_SINGLE_SHORT_OK;
    }
    memset(&init, 0, sizeof init);
    init.pEndpoints = u->ep;
    init.ep_index_max = u->xinput ? N_RD + 1 : N_RD;
    if (ioctl(u->fd, USB_FS_INIT, &init) != 0) {
        log_line("usb: %s: init errno %d", u->path, errno);
        goto fail;
    }
    for (i = 0; i < u->nreads; i++) {
        struct usb_fs_open op;
        memset(&op, 0, sizeof op);
        op.ep_index = (uint8_t)i;
        op.ep_no = u->ep_in;
        op.max_bufsize = u->mps;
        op.max_frames = 1;
        if (ioctl(u->fd, USB_FS_OPEN, &op) != 0) {
            log_line("usb: %s: open endpoint %#x errno %d", u->path, u->ep_in, errno);
            usbpad_close(u);
            return NULL;
        }
    }
    if (u->xinput) {
        struct usb_fs_open op = {0};
        struct usb_fs_start start = {0};
        op.ep_index = N_RD; op.ep_no = u->ep_out;
        op.max_bufsize = 32; op.max_frames = 1;
        u->enable[0] = 1; u->enable[1] = 3; u->enable[2] = 0x0e;
        u->bufp[N_RD][0] = u->enable; u->lenp[N_RD][0] = 3;
        u->ep[N_RD].ppBuffer = u->bufp[N_RD];
        u->ep[N_RD].pLength = u->lenp[N_RD];
        u->ep[N_RD].nFrames = 1; u->ep[N_RD].timeout = 500;
        start.ep_index = N_RD;
        if (ioctl(u->fd, USB_FS_OPEN, &op) != 0 ||
            ioctl(u->fd, USB_FS_START, &start) != 0) {
            log_line("usb: XInput enable failed errno %d", errno);
            usbpad_close(u); return NULL;
        }
        u->enable_pending = 1; u->t_enable = now_ms();
        log_line("usb: XInput enable 01 03 0e queued on OUT %#x", u->ep_out);
    }
    u->t_alive = now_ms();
    log_line("usb: %s %04x:%04x \"%s\", reading endpoint %#x (%u bytes)", u->path, UGETW(dd.idVendor),
             UGETW(dd.idProduct), u->name, u->ep_in, u->mps);
    return u;

fail:
    if (u->fd >= 0) close(u->fd);
    free(u);
    return NULL;
}

usbpad *usbpad_open(void)
{
    int bus, addr;

    for (bus = 0; bus < 4; bus++)
        for (addr = 2; addr < 16; addr++) {
            char path[24];
            usbpad *u;
            if (bus == 0 && addr == 2) continue;        /* the internal Bluetooth chip: never touched */
            snprintf(path, sizeof path, "/dev/ugen%d.%d", bus, addr);
            if (access(path, F_OK) != 0) continue;
            if ((u = try_node(bus, addr))) return u;
        }
    return NULL;
}

const char *usbpad_name(const usbpad *u)
{
    return u && u->name[0] ? u->name : "USB pad";
}

int usbpad_poll(usbpad *u, pad_state *st)
{
    struct usb_fs_complete done;
    int i, got = 0;
    long now = now_ms();

    for (i = 0; i < u->nreads; i++)
        if (!u->busy[i] && start_read(u, i) < 0) {
            log_line("usb: %s: gone (start errno %d)", u->path, errno);
            return -1;
        }
    for (;;) {
        memset(&done, 0, sizeof done);
        if (ioctl(u->fd, USB_FS_COMPLETE, &done) != 0) {
            if (errno != EBUSY && dead_errno(errno)) {
                log_line("usb: %s: gone (complete errno %d)", u->path, errno);
                return -1;
            }
            break;
        }
        i = done.ep_index;
        if (u->xinput && i == N_RD) {
            u->enable_pending = 0;
            log_line("usb: XInput enable status %u length %u", u->ep[i].status, u->lenp[i][0]);
            if (u->ep[i].status || u->lenp[i][0] != 3) return -1;
            continue;
        }
        if (i >= u->nreads) continue;
        u->busy[i] = 0;
        if (u->ep[i].status) continue;              /* cancelled or stalled: read again */
        if (!u->ep[i].aFrames) continue;
        if (u->lenp[i][0] > u->mps) continue;
        if ((u->xinput ? usbpad_parse_xinput : usbpad_parse_8bitdo)
            (u->buf[i], (int)u->lenp[i][0], st)) {
            if (!u->logged_first) {
                u->logged_first = 1;
                log_line("usb: first controller report");
            }
            got = 1;
        }
    }
    if (u->enable_pending && now - u->t_enable > 1500) {
        log_line("usb: XInput enable completion timed out"); return -1;
    }
    /* A re-enumeration does not always fail the pending reads at once.
     * (USB_GET_DEVICEINFO fails on the 8BitDo dongle, so it is not used.) */
    if (now - u->t_alive >= T_ALIVE) {
        struct usb_device_descriptor dd;
        u->t_alive = now;
        if (access(u->path, F_OK) != 0 || ioctl(u->fd, USB_GET_DEVICE_DESC, &dd) != 0 ||
            UGETW(dd.idVendor) != VID_8BITDO || UGETW(dd.idProduct) != u->pid) {
            log_line("usb: %s: gone (errno %d)", u->path, errno);
            return -1;
        }
    }
    return got;
}

void usbpad_close(usbpad *u)
{
    struct usb_fs_uninit un;

    if (!u) return;
    if (u->fd >= 0) {
        memset(&un, 0, sizeof un);
        ioctl(u->fd, USB_FS_UNINIT, &un);
        close(u->fd);
    }
    free(u);
}
