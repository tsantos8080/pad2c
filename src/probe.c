/* pad2c probe -- diagnostic look at the console's USB devices.
 *
 * For 8BitDo devices on /dev/ugenB.A (the internal Bluetooth chip ugen0.2
 * is left alone on purpose): device descriptor, name, full
 * configuration descriptor, which kernel driver holds each interface, the
 * HID report descriptor of HID interfaces, and then 60 seconds of whatever
 * its interrupt IN endpoints send (press buttons and move the sticks then).
 * Also copies what /dev/klog holds, to see why another payload died.
 *
 * Everything goes to stdout (the loader's socket) and /data/pad2c/probe.log.
 * Rescans for up to 120 seconds, skipping the idle 301c identity and reading
 * interrupt IN endpoints regardless of interface class, including XInput.
 * No driver detach. For 2dc8:310a with the confirmed XInput topology, sends
 * the candidate XInput enable command 01 03 0e to endpoint 05 once per session.
 */
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>

#include <ps5/kernel.h>

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define DIR_PATH "/data/pad2c"
#define LOG_PATH DIR_PATH "/probe.log"
#define KLOG_PATH DIR_PATH "/klog.txt"
#ifndef READ_SECONDS
#define READ_SECONDS 60
#endif
#define MAX_EPS 8
#define WAIT_SECONDS 120
#define VID_8BITDO 0x2dc8
#define PID_IDLE 0x301c
#define PID_XINPUT 0x310a

static uint16_t seen_pid[4][10];
static int captured;

typedef struct {
    char reserved[45];
    char message[3075];
} notify_request;
int sceKernelSendNotificationRequest(int, notify_request *, size_t, int);

static FILE *g_log;

static void out(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
    if (g_log) {
        va_start(ap, fmt);
        vfprintf(g_log, fmt, ap);
        va_end(ap);
        fprintf(g_log, "\n");
        fflush(g_log);
    }
}

static void notify(const char *msg)
{
    notify_request req;

    memset(&req, 0, sizeof req);
    snprintf(req.message, sizeof req.message, "%s", msg);
    sceKernelSendNotificationRequest(0, &req, sizeof req, 0);
}

static long now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void hexdump(const char *tag, const uint8_t *p, int n)
{
    char line[3 * 32 + 1];
    int i, k;

    for (i = 0; i < n; i += 32) {
        int m = n - i < 32 ? n - i : 32;
        for (k = 0; k < m; k++) snprintf(line + 3 * k, 4, "%02x ", p[i + k]);
        line[3 * m] = '\0';
        out("%s[%03d] %s", tag, i, line);
    }
}

/* ---- /dev/klog ------------------------------------------------------------- */

static void copy_klog(void)
{
    char buf[4096];
    int fd = open("/dev/klog", O_RDONLY | O_NONBLOCK);
    FILE *f;
    long total = 0;
    ssize_t n;

    if (fd < 0) {
        out("klog: cannot open /dev/klog (errno %d)", errno);
        return;
    }
    f = fopen(KLOG_PATH, "w");
    while ((n = read(fd, buf, sizeof buf)) > 0 && total < 4 * 1024 * 1024) {
        if (f) fwrite(buf, 1, (size_t)n, f);
        total += n;
    }
    if (f) fclose(f);
    close(fd);
    out("klog: %ld bytes copied to %s", total, KLOG_PATH);
}

/* ---- one USB device ---------------------------------------------------------- */

typedef struct {
    uint8_t addr;
    uint16_t mps;
    int iface;
} ep_in;

static int parse_config(const uint8_t *cfg, int len, ep_in *eps, int *hid_iface, int *hid_len, int nhid_max,
                        int *nhid_out)
{
    int off = 0, n = 0, iface = -1, iface_class = 0, alt = 0, nhid = 0;

    while (off + 2 <= len) {
        int dl = cfg[off];
        const uint8_t *d = cfg + off;

        if (dl < 2 || off + dl > len) break;
        if (d[1] == 0x04 && dl >= 9) {                  /* interface */
            iface = d[2];
            alt = d[3];
            iface_class = d[5];
            out("  interface %d alt %d: class %02x/%02x/%02x, %d endpoint(s)", d[2], d[3], d[5], d[6], d[7], d[4]);
            if (d[5] == 0xff && d[6] == 0x5d && d[7] == 0x01)
                out("  XInput interface detected");
        } else if (d[1] == 0x21 && dl >= 9 && iface_class == 0x03) {   /* HID descriptor */
            int rlen = d[7] | d[8] << 8;
            out("  HID descriptor: report descriptor of %d bytes", rlen);
            if (nhid < nhid_max) {
                hid_iface[nhid] = iface;
                hid_len[nhid] = rlen;
                nhid++;
            }
        } else if (d[1] == 0x05 && dl >= 7) {           /* endpoint */
            static const char *const type[] = { "control", "isochronous", "bulk", "interrupt" };
            uint16_t mps = (uint16_t)(d[4] | d[5] << 8);
            out("  endpoint %#04x %s %s, %u bytes, interval %u", d[2], d[2] & 0x80 ? "IN" : "OUT",
                type[d[3] & 3], mps, d[6]);
            if (alt == 0 && (d[2] & 0x80) && (d[3] & 3) == 3 && mps && n < MAX_EPS) {
                eps[n].addr = d[2];
                eps[n].mps = mps;
                eps[n].iface = iface;
                n++;
            }
        } else {
            out("  descriptor type %#04x, %d bytes", d[1], dl);
        }
        off += dl;
    }
    *nhid_out = nhid;
    return n;
}

static void get_report_descriptor(int fd, int iface, int len)
{
    struct usb_ctl_request req;
    uint8_t buf[1024];

    if (len <= 0 || len > (int)sizeof buf) len = sizeof buf;
    memset(&req, 0, sizeof req);
    memset(buf, 0, sizeof buf);
    req.ucr_data = buf;
    req.ucr_flags = 0x0004;                     /* USB_SHORT_XFER_OK (usbdi.h, kernel-only header) */
    req.ucr_request.bmRequestType = UT_READ_INTERFACE;
    req.ucr_request.bRequest = UR_GET_DESCRIPTOR;
    USETW(req.ucr_request.wValue, 0x2200);
    USETW(req.ucr_request.wIndex, iface);
    USETW(req.ucr_request.wLength, len);
    if (ioctl(fd, USB_DO_REQUEST, &req) != 0) {
        out("  report descriptor of interface %d: errno %d", iface, errno);
        return;
    }
    out("  report descriptor of interface %d, %d bytes:", iface, req.ucr_actlen);
    hexdump("  rdesc", buf, req.ucr_actlen);
}

/* Reads every interrupt IN endpoint for a while, logging each report that
 * differs from the previous one on that endpoint. */
static void read_reports(int fd, const ep_in *eps, int n, uint16_t pid, int enable)
{
    struct usb_fs_endpoint ep[MAX_EPS + 1];
    static uint8_t buf[MAX_EPS][512], last[MAX_EPS][512];
    void *bufp[MAX_EPS + 1][1];
    uint32_t lenp[MAX_EPS + 1][1];
    uint8_t enable_cmd[] = { 0x01, 0x03, 0x0e };
    int enable_pending = 0;
    int lastlen[MAX_EPS], opened[MAX_EPS], busy[MAX_EPS], count[MAX_EPS];
    struct usb_fs_init init;
    long end, last_check = 0;
    int i;

    memset(ep, 0, sizeof ep);
    memset(lastlen, 0, sizeof lastlen);
    memset(opened, 0, sizeof opened);
    memset(busy, 0, sizeof busy);
    memset(count, 0, sizeof count);
    for (i = 0; i < n; i++) {
        bufp[i][0] = buf[i];
        lenp[i][0] = eps[i].mps > 512 ? 512 : eps[i].mps;
        ep[i].ppBuffer = bufp[i];
        ep[i].pLength = lenp[i];
        ep[i].nFrames = 1;
        ep[i].flags = USB_FS_FLAG_SINGLE_SHORT_OK;
    }
    memset(&init, 0, sizeof init);
    init.pEndpoints = ep;
    init.ep_index_max = n + enable;
    if (ioctl(fd, USB_FS_INIT, &init) != 0) {
        out("  reports: USB_FS_INIT errno %d", errno);
        return;
    }
    for (i = 0; i < n; i++) {
        struct usb_fs_open op;
        memset(&op, 0, sizeof op);
        op.ep_index = (uint8_t)i;
        op.ep_no = eps[i].addr;
        op.max_bufsize = eps[i].mps > 512 ? 512 : eps[i].mps;
        op.max_frames = 1;
        opened[i] = ioctl(fd, USB_FS_OPEN, &op) == 0;
        out("  reports: open endpoint %#04x (interface %d): %s (errno %d)", eps[i].addr, eps[i].iface,
            opened[i] ? "ok" : "FAILED", opened[i] ? 0 : errno);
    }

    if (enable) {
        struct usb_fs_open op;
        struct usb_fs_start start;
        memset(&op, 0, sizeof op);
        op.ep_index = (uint8_t)n;
        op.ep_no = 0x05;
        op.max_bufsize = 32;
        op.max_frames = 1;
        if (ioctl(fd, USB_FS_OPEN, &op) != 0) {
            out("  enable: open OUT 0x05 failed errno %d", errno);
        } else {
            bufp[n][0] = enable_cmd;
            lenp[n][0] = sizeof enable_cmd;
            ep[n].ppBuffer = bufp[n];
            ep[n].pLength = lenp[n];
            ep[n].nFrames = 1;
            ep[n].timeout = 500;
            memset(&start, 0, sizeof start);
            start.ep_index = (uint8_t)n;
            enable_pending = ioctl(fd, USB_FS_START, &start) == 0;
            if (enable_pending) out("  enable: queued candidate 01 03 0e on OUT 0x05");
            else out("  enable: START failed errno %d", errno);
        }
    }

    out(">>> PRESS BUTTONS, TRIGGERS AND MOVE BOTH STICKS NOW (%d s) <<<", READ_SECONDS);
    notify("pad2c probe: press every button, one at a time (60 s)");
    end = now_ms() + READ_SECONDS * 1000;
    while (now_ms() < end) {
        struct usb_fs_complete done;
        int any = 0;

        if (now_ms() - last_check >= 250) {
            struct usb_device_descriptor dd;
            last_check = now_ms();
            if (ioctl(fd, USB_GET_DEVICE_DESC, &dd) != 0) {
                out("  reports: device unavailable (errno %d); rescanning", errno);
                break;
            }
            if (UGETW(dd.idVendor) != VID_8BITDO || UGETW(dd.idProduct) != pid) {
                out("  reports: USB identity changed; rescanning");
                break;
            }
        }

        for (i = 0; i < n; i++) {
            if (!opened[i] || busy[i]) continue;
            struct usb_fs_start st;
            lenp[i][0] = eps[i].mps > 512 ? 512 : eps[i].mps;
            ep[i].nFrames = 1;
            ep[i].aFrames = 0;
            ep[i].status = 0;
            memset(&st, 0, sizeof st);
            st.ep_index = (uint8_t)i;
            if (ioctl(fd, USB_FS_START, &st) == 0) busy[i] = 1;
            else {
                out("  reports: start %#04x errno %d, giving up on it", eps[i].addr, errno);
                opened[i] = 0;
            }
        }
        for (;;) {
            memset(&done, 0, sizeof done);
            if (ioctl(fd, USB_FS_COMPLETE, &done) != 0) {
                if (errno != EBUSY && errno != EAGAIN) {
                    out("  reports: COMPLETE errno %d; rescanning", errno);
                    end = 0;
                }
                break;
            }
            any = 1;
            i = done.ep_index;
            if (enable && i == n) {
                enable_pending = 0;
                out("  enable: completed status %u, frames %u, length %u",
                    (unsigned)ep[n].status, (unsigned)ep[n].aFrames, (unsigned)lenp[n][0]);
                continue;
            }
            if (i >= n) continue;
            busy[i] = 0;
            if (ep[i].status) {
                out("  reports: %#04x finished with status %u", eps[i].addr, (unsigned)ep[i].status);
                continue;
            }
            if (ep[i].aFrames) {
                int len = (int)lenp[i][0];
                count[i]++;
                if (len < 0 || len > (int)sizeof buf[i]) {
                    out("  reports: invalid length %d on endpoint %#04x", len, eps[i].addr);
                    continue;
                }
                if (!len) {
                    if (count[i] == 1) out("  reports: endpoint %#04x returned an empty transfer", eps[i].addr);
                    continue;
                }
                captured = 1;
                if (len != lastlen[i] || memcmp(buf[i], last[i], (size_t)len) != 0) {
                    char tag[48];
                    memcpy(last[i], buf[i], (size_t)len);
                    lastlen[i] = len;
                    snprintf(tag, sizeof tag, "  ep%02x t=%05ld", eps[i].addr, READ_SECONDS * 1000 - (end - now_ms()));
                    hexdump(tag, buf[i], len);
                }
            }
        }
        if (!any) usleep(1000);
    }
    for (i = 0; i < n; i++) out("  reports: endpoint %#04x delivered %d report(s)", eps[i].addr, count[i]);
    if (enable_pending) out("  enable: no completion received before capture ended");
    {
        struct usb_fs_uninit un;
        memset(&un, 0, sizeof un);
        ioctl(fd, USB_FS_UNINIT, &un);
    }
}

static void probe_device(int bus, int addr)
{
    char path[32];
    struct usb_device_descriptor dd;
    struct usb_device_info di;
    struct usb_gen_descriptor gd;
    static uint8_t cfg[2048];
    ep_in eps[MAX_EPS];
    int hid_iface[4], hid_len[4], nhid = 0, neps = 0;
    int fd, i, total = 0, enable = 0;

    snprintf(path, sizeof path, "/dev/ugen%d.%d", bus, addr);
    if (access(path, F_OK) != 0) {
        seen_pid[bus][addr] = 0;
        return;
    }
    if (bus == 0 && addr == 2) {
        out("%s: internal Bluetooth chip, not touched", path);
        return;
    }
    fd = open(path, O_RDWR);
    if (fd < 0) {
        seen_pid[bus][addr] = 0;
        return;
    }
    if (ioctl(fd, USB_GET_DEVICE_DESC, &dd) != 0) {
        out("%s: device descriptor errno %d", path, errno);
        close(fd);
        return;
    }
    if (UGETW(dd.idVendor) != VID_8BITDO) {
        close(fd);
        return;
    }
    if (seen_pid[bus][addr] == UGETW(dd.idProduct)) {
        close(fd);
        return;
    }
    seen_pid[bus][addr] = UGETW(dd.idProduct);
    out("%s: %04x:%04x class %02x/%02x/%02x USB %04x", path, UGETW(dd.idVendor), UGETW(dd.idProduct),
        dd.bDeviceClass, dd.bDeviceSubClass, dd.bDeviceProtocol, UGETW(dd.bcdUSB));
    if (dd.bDeviceClass == 0x09) {
        out("  a hub, skipped");
        close(fd);
        return;
    }
    memset(&di, 0, sizeof di);
    if (ioctl(fd, USB_GET_DEVICEINFO, &di) == 0)
        out("  \"%.60s\" by \"%.60s\", speed %u, config %u", di.udi_product, di.udi_vendor, di.udi_speed,
            di.udi_config_no);

    memset(&gd, 0, sizeof gd);
    gd.ugd_data = cfg;
    gd.ugd_maxlen = sizeof cfg;
    gd.ugd_config_index = 0xFF;
    if (ioctl(fd, USB_GET_FULL_DESC, &gd) == 0) {
        total = gd.ugd_actlen < (int)sizeof cfg ? gd.ugd_actlen : (int)sizeof cfg;
        out("  configuration descriptor, %d bytes", total);
        hexdump("  cfg", cfg, total);
        neps = parse_config(cfg, total, eps, hid_iface, hid_len, 4, &nhid);
    } else {
        out("  configuration descriptor errno %d", errno);
    }
    for (i = 0; i < 8; i++) {
        char name[64];
        memset(&gd, 0, sizeof gd);
        memset(name, 0, sizeof name);
        gd.ugd_data = name;
        gd.ugd_maxlen = sizeof name - 1;
        gd.ugd_iface_index = (uint8_t)i;
        if (ioctl(fd, USB_GET_IFACE_DRIVER, &gd) == 0) out("  interface %d held by driver \"%s\"", i, name);
    }
    for (i = 0; i < nhid; i++) get_report_descriptor(fd, hid_iface[i], hid_len[i]);
    if (UGETW(dd.idProduct) == PID_IDLE) {
        out("  IDLE: waiting for active USB identity (expected XInput 2dc8:310a)");
        close(fd);
        return;
    }
    if (UGETW(dd.idProduct) == PID_XINPUT)
        out("  Ultimate 2C XInput: testing candidate enable command before capture");
    /* Only enable the exact interface/endpoint topology captured on WUKONG. */
    if (UGETW(dd.idProduct) == PID_XINPUT && neps > 0 &&
        eps[0].iface == 0 && eps[0].addr == 0x84 && eps[0].mps == 32) {
        int off = 0, xinput = 0;
        while (off + 2 <= total) {
            const uint8_t *d = cfg + off;
            int dl = d[0];
            if (dl < 2 || off + dl > total) break;
            if (d[1] == 4 && dl >= 9)
                xinput = d[2] == 0 && d[3] == 0 && d[5] == 0xff && d[6] == 0x5d && d[7] == 1;
            if (xinput && d[1] == 5 && dl >= 7 && d[2] == 5 &&
                (d[3] & 3) == 3 && d[4] == 32 && d[5] == 0) enable = 1;
            off += dl;
        }
    }
    if (neps > 0) read_reports(fd, eps, neps, UGETW(dd.idProduct), enable);
    else out("  no interrupt IN endpoint");
    seen_pid[bus][addr] = 0; /* Retry after a disconnect or an empty capture. */
    close(fd);
}

int main(void)
{
    uint32_t fw;
    int bus, addr;
    long deadline;

    mkdir(DIR_PATH, 0777);
    g_log = fopen(LOG_PATH, "w");
    fw = kernel_get_fw_version();
    out("pad2c probe 3 (XInput enable capture), firmware %x.%02x (raw %#x), pid %d", (unsigned)(fw >> 24) & 0xFF,
        (unsigned)(fw >> 16) & 0xFF, (unsigned)fw, (int)getpid());
    notify("pad2c probe: starting");
    copy_klog();
    out("Waiting up to %d s for active 8BitDo; capture lasts %d s. Run without other USB payloads.", WAIT_SECONDS, READ_SECONDS);
    deadline = now_ms() + WAIT_SECONDS * 1000;
    while (!captured && now_ms() < deadline) {
        for (bus = 0; bus < 4 && !captured; bus++)
            for (addr = 1; addr < 10 && !captured; addr++) probe_device(bus, addr);
        if (!captured) usleep(250000);
    }
    if (!captured) out("No nonempty input reports captured; initialization or USB mode needs investigation.");
    out("done");
    notify("pad2c probe: done");
    if (g_log) fclose(g_log);
    return 0;
}
