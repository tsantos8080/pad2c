/* Reads supported 8BitDo HID and Ultimate 2C XInput USB interfaces.
 * The idle dongle is skipped; re-enumeration causes close and rescan. */
#ifndef PAD2C_USBPAD_H
#define PAD2C_USBPAD_H

#include "pad.h"

typedef struct usbpad usbpad;

/* Looks for a supported pad. NULL if there is none right now. */
usbpad *usbpad_open(void);

/* Takes what came in. Returns 1 with *st updated when a controller report
 * arrived, 0 when nothing new, -1 when the device is gone (close it). */
int usbpad_poll(usbpad *u, pad_state *st);

void usbpad_close(usbpad *u);
const char *usbpad_name(const usbpad *u);

/* The 8BitDo report 1 to a DualSense state. Returns 0 if it is not one. */
int usbpad_parse_xinput(const unsigned char *r, int n, pad_state *st);
int usbpad_parse_8bitdo(const unsigned char *r, int n, pad_state *st);

#endif
