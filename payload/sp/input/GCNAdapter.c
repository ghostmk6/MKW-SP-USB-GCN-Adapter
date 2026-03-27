#include "GCNAdapter.h"

#include <sp/storage/Usb.h>

#include <revolution.h>
#include <revolution/ios.h>

#include <string.h>

#define WUP028_VID 0x057E
#define WUP028_PID 0x0337

#define EP_IN 0x81
#define EP_OUT 0x02

#define CMD_INIT 0x13
#define CMD_RUMBLE 0x11

#define REPORT_SIZE 37
#define REPORT_ID 0x21
#define PORT_BLOCK_BYTES 9

#define STATUS_CONNECTED 0x10
#define STATUS_WIRELESS 0x20

#define BTN0_A (1 << 0)
#define BTN0_B (1 << 1)
#define BTN0_X (1 << 2)
#define BTN0_Y (1 << 3)
#define BTN0_DPAD_LEFT (1 << 4)
#define BTN0_DPAD_RIGHT (1 << 5)
#define BTN0_DPAD_DOWN (1 << 6)
#define BTN0_DPAD_UP (1 << 7)

#define BTN1_START (1 << 0)
#define BTN1_Z (1 << 1)
#define BTN1_R (1 << 2)
#define BTN1_L (1 << 3)

#define TRIGGER_THRESHOLD 170

static bool s_connected = false;
static u32 s_deviceId = 0;
static GCNPortState s_ports[GCN_PORT_COUNT];

static u8 s_reportBuf[REPORT_SIZE] __attribute__((aligned(32)));
static u8 s_rumbleBuf[5] __attribute__((aligned(32)));
static u8 s_initBuf[1] __attribute__((aligned(32)));

static bool GCNAdapter_onDeviceAdd(const UsbDeviceInfo *info) {
    if (info->deviceDescriptor.vendorId != WUP028_VID ||
            info->deviceDescriptor.productId != WUP028_PID) {
        return false;
    }

    s_deviceId = info->id;
    SP_LOG("GCNAdapter: WUP-028 detected (id=%08x)", s_deviceId);

    s_initBuf[0] = CMD_INIT;
    if (!Usb_bulkTransfer(s_deviceId, EP_OUT, sizeof(s_initBuf), s_initBuf)) {
        SP_LOG("GCNAdapter: Failed to send init command");
        return false;
    }

    memset(s_ports, 0, sizeof(s_ports));
    s_connected = true;
    SP_LOG("GCNAdapter: Initialised successfully");
    return true;
}

static void GCNAdapter_onDeviceRemove(u32 id) {
    if (id != s_deviceId) {
        return;
    }
    SP_LOG("GCNAdapter: WUP-028 disconnected");
    s_connected = false;
    s_deviceId = 0;
    memset(s_ports, 0, sizeof(s_ports));
}

static UsbHandler s_handler = {
        .onDeviceAdd = GCNAdapter_onDeviceAdd,
        .onDeviceRemove = GCNAdapter_onDeviceRemove,
        .next = NULL,
};

void GCNAdapter_init(void) {
    Usb_addHandler(&s_handler);
    SP_LOG("GCNAdapter: Handler registered");
}

static void parseReport(const u8 *buf) {
    for (u32 i = 0; i < GCN_PORT_COUNT; i++) {
        const u8 *b = buf + 1 + i * PORT_BLOCK_BYTES;
        GCNPortState *p = &s_ports[i];

        u8 status = b[0];
        p->connected = (status & STATUS_CONNECTED) != 0;
        p->wireless = (status & STATUS_WIRELESS) != 0;

        if (!p->connected) {
            bool savedRumble = p->rumble;
            memset(p, 0, sizeof(*p));
            p->rumble = savedRumble;
            continue;
        }

        u8 btn0 = b[1];
        u8 btn1 = b[2];

        p->a = (btn0 & BTN0_A) != 0;
        p->b = (btn0 & BTN0_B) != 0;
        p->x = (btn0 & BTN0_X) != 0;
        p->y = (btn0 & BTN0_Y) != 0;
        p->dpadLeft = (btn0 & BTN0_DPAD_LEFT) != 0;
        p->dpadRight = (btn0 & BTN0_DPAD_RIGHT) != 0;
        p->dpadDown = (btn0 & BTN0_DPAD_DOWN) != 0;
        p->dpadUp = (btn0 & BTN0_DPAD_UP) != 0;

        p->start = (btn1 & BTN1_START) != 0;
        p->z = (btn1 & BTN1_Z) != 0;
        p->r = (btn1 & BTN1_R) != 0;
        p->l = (btn1 & BTN1_L) != 0;

        p->stickX = b[3];
        p->stickY = b[4];
        p->cstickX = b[5];
        p->cstickY = b[6];
        p->triggerL = b[7];
        p->triggerR = b[8];
    }
}

void GCNAdapter_poll(void) {
    if (!s_connected) {
        return;
    }

    if (!Usb_bulkTransfer(s_deviceId, EP_IN, REPORT_SIZE, s_reportBuf)) {
        SP_LOG("GCNAdapter: poll read failed");
        s_connected = false;
        memset(s_ports, 0, sizeof(s_ports));
        return;
    }

    if (s_reportBuf[0] != REPORT_ID) {
        return;
    }

    parseReport(s_reportBuf);
}

void GCNAdapter_sendRumble(void) {
    if (!s_connected) {
        return;
    }

    s_rumbleBuf[0] = CMD_RUMBLE;
    for (u32 i = 0; i < GCN_PORT_COUNT; i++) {
        s_rumbleBuf[1 + i] = s_ports[i].rumble ? 0x01 : 0x00;
        s_ports[i].rumble = false;
    }

    Usb_bulkTransfer(s_deviceId, EP_OUT, sizeof(s_rumbleBuf), s_rumbleBuf);
}

bool GCNAdapter_isConnected(void) {
    return s_connected;
}

const GCNPortState *GCNAdapter_getPort(u32 port) {
    if (port >= GCN_PORT_COUNT) {
        return NULL;
    }
    return &s_ports[port];
}
