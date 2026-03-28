#include "GCNAdapter.h"

#include <revolution.h>
#include <revolution/ios.h>

#include <string.h>

// GCNAdapter.c
// Direct IOS USB HID implementation for the WUP-028 GameCube controller adapter.
//
// Uses /dev/usb/hid5 (IOS57+, vWii) with fallback to /dev/usb/hid (IOS37/53).
// This bypasses MKW-SP's Usb.c storage stack entirely and talks directly to
// the IOS HID module, which is the only approach that works on vWii.
//
// Protocol reference (from Chadderz wup-028-bslug):
//   HID5 flow:
//     1) IOS_Ioctl GET_VERSION    -> must return 0x00050001
//     2) IOS_Ioctl GET_DEVICE_CHANGE -> returns device list
//     3) IOS_Ioctl ATTACH_FINISH
//     4) Find WUP-028 (VID 057E PID 0337)
//     5) IOS_Ioctl SET_RESUME     -> wake device
//     6) IOS_Ioctl GET_DEVICE_PARAMETERS -> required even if unused
//     7) IOS_Ioctl INTERRUPT (OUT) -> send 0x13 init command
//     8) IOS_Ioctl INTERRUPT (IN)  -> poll 37-byte report each frame
//
//   HID4 flow:
//     1) IOS_Ioctl GET_VERSION    -> must return 0x00040001
//     2) IOS_Ioctl GET_DEVICE_CHANGE -> returns device list
//     3) Find WUP-028
//     4) IOS_Ioctl INTERRUPT_OUT  -> send 0x13 init command
//     5) IOS_Ioctl INTERRUPT_IN   -> poll 37-byte report each frame

// ---------------------------------------------------------------------------
// HID5 ioctl numbers (/dev/usb/hid5, IOS57+, used by vWii)
// ---------------------------------------------------------------------------
#define HID5_GET_VERSION 0x00
#define HID5_GET_DEVICE_CHANGE 0x01
#define HID5_SET_RESUME 0x02
#define HID5_GET_DEVICE_PARAMETERS 0x03
#define HID5_ATTACH_FINISH 0x06
#define HID5_INTERRUPT 0x09

#define HID5_VERSION 0x00050001
#define HID5_DEVICE_CHANGE_SIZE 0x20 // max devices

// HID5 device entry layout
typedef struct {
    u32 id;
    u16 vid;
    u16 pid;
    u8 interfaceNumber;
    u8 alternateSetting;
    u8 _0a[2];
} __attribute__((packed)) Hid5DeviceEntry;

// HID5 interrupt transfer command block
typedef struct {
    u32 deviceId;
    u8 endpoint;
    u8 _05[3];
    u32 length;
    void *data;
} __attribute__((packed)) Hid5InterruptCmd;

// ---------------------------------------------------------------------------
// HID4 ioctl numbers (/dev/usb/hid, IOS37/53)
// ---------------------------------------------------------------------------
#define HID4_GET_VERSION 0x00
#define HID4_GET_DEVICE_CHANGE 0x01
#define HID4_INTERRUPT_OUT 0x02
#define HID4_INTERRUPT_IN 0x03

#define HID4_VERSION 0x00040001
#define HID4_DEVICE_CHANGE_WORDS 0x180

// HID4 interrupt command block (used for both IN and OUT)
typedef struct {
    u32 deviceId;
    u8 endpoint;
    u8 _05[3];
    u32 length;
    void *data;
} __attribute__((packed)) Hid4InterruptCmd;

// ---------------------------------------------------------------------------
// WUP-028 constants
// ---------------------------------------------------------------------------
#define WUP028_VID 0x057E
#define WUP028_PID 0x0337
#define EP_OUT 0x02
#define EP_IN 0x81
#define CMD_INIT 0x13
#define CMD_RUMBLE 0x11
#define REPORT_SIZE 37
#define REPORT_ID 0x21
#define PORT_BLOCK 9
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

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

typedef enum {
    HID_NONE = 0,
    HID_VERSION_4,
    HID_VERSION_5,
} HidVersion;

static HidVersion s_hidVersion = HID_NONE;
static s32 s_fd = -1;
static bool s_connected = false;
static u32 s_deviceId = 0;
static GCNPortState s_ports[GCN_PORT_COUNT];

// All IOS DMA buffers must be 32-byte aligned
static u8 s_versionBuf[32] __attribute__((aligned(32)));
static u32 s_hid4DevChangeBuf[HID4_DEVICE_CHANGE_WORDS] __attribute__((aligned(32)));
static Hid5DeviceEntry s_hid5DevChangeBuf[HID5_DEVICE_CHANGE_SIZE] __attribute__((aligned(32)));
static u8 s_reportBuf[REPORT_SIZE] __attribute__((aligned(32)));
static u8 s_cmdBuf[32] __attribute__((aligned(32)));
static u8 s_rumbleBuf[8] __attribute__((aligned(32)));
static u8 s_paramBuf[64] __attribute__((aligned(32)));
static Hid5InterruptCmd s_hid5Cmd __attribute__((aligned(32)));
static Hid4InterruptCmd s_hid4Cmd __attribute__((aligned(32)));

// ---------------------------------------------------------------------------
// Report parsing (shared between HID4 and HID5)
// ---------------------------------------------------------------------------

static void parseReport(const u8 *buf) {
    for (u32 i = 0; i < GCN_PORT_COUNT; i++) {
        const u8 *b = buf + 1 + i * PORT_BLOCK;
        GCNPortState *p = &s_ports[i];
        bool savedRumble = p->rumble;

        u8 status = b[0];
        p->connected = (status & STATUS_CONNECTED) != 0;
        p->wireless = (status & STATUS_WIRELESS) != 0;

        if (!p->connected) {
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

// ---------------------------------------------------------------------------
// HID5 implementation (/dev/usb/hid5, vWii / IOS57+)
// ---------------------------------------------------------------------------

static bool hid5Init(void) {
    s_fd = IOS_Open("/dev/usb/hid5", 0);
    if (s_fd < 0) {
        return false;
    }

    // 1) Check version
    if (IOS_Ioctl(s_fd, HID5_GET_VERSION, NULL, 0, s_versionBuf, sizeof(s_versionBuf)) < 0) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }
    u32 ver = (u32)s_versionBuf[0] << 24 | (u32)s_versionBuf[1] << 16 | (u32)s_versionBuf[2] << 8 |
            (u32)s_versionBuf[3];
    if (ver != HID5_VERSION) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }

    // 2) Get device list
    s32 count = IOS_Ioctl(s_fd, HID5_GET_DEVICE_CHANGE, NULL, 0, s_hid5DevChangeBuf,
            sizeof(s_hid5DevChangeBuf));
    if (count < 0) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }

    // 3) Attach finish (required by HID5)
    IOS_Ioctl(s_fd, HID5_ATTACH_FINISH, NULL, 0, NULL, 0);

    // 4) Find WUP-028
    s_deviceId = 0;
    for (s32 i = 0; i < count; i++) {
        if (s_hid5DevChangeBuf[i].vid == WUP028_VID && s_hid5DevChangeBuf[i].pid == WUP028_PID) {
            s_deviceId = s_hid5DevChangeBuf[i].id;
            break;
        }
    }
    if (s_deviceId == 0) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }

    // 5) Resume (wake) the device
    memset(s_cmdBuf, 0, sizeof(s_cmdBuf));
    *(u32 *)s_cmdBuf = s_deviceId;
    s_cmdBuf[4] = 0; // not suspended
    IOS_Ioctl(s_fd, HID5_SET_RESUME, s_cmdBuf, 8, NULL, 0);

    // 6) Get device parameters (required even if we don't use the result)
    memset(s_cmdBuf, 0, sizeof(s_cmdBuf));
    *(u32 *)s_cmdBuf = s_deviceId;
    IOS_Ioctl(s_fd, HID5_GET_DEVICE_PARAMETERS, s_cmdBuf, 8, s_paramBuf, sizeof(s_paramBuf));

    // 7) Send init command 0x13
    s_cmdBuf[0] = CMD_INIT;
    memset(&s_hid5Cmd, 0, sizeof(s_hid5Cmd));
    s_hid5Cmd.deviceId = s_deviceId;
    s_hid5Cmd.endpoint = EP_OUT;
    s_hid5Cmd.length = 1;
    s_hid5Cmd.data = s_cmdBuf;
    if (IOS_Ioctl(s_fd, HID5_INTERRUPT, &s_hid5Cmd, sizeof(s_hid5Cmd), NULL, 0) < 0) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }

    s_hidVersion = HID_VERSION_5;
    SP_LOG("GCNAdapter: HID5 init OK (id=%08x)", s_deviceId);
    return true;
}

static void hid5Poll(void) {
    memset(&s_hid5Cmd, 0, sizeof(s_hid5Cmd));
    s_hid5Cmd.deviceId = s_deviceId;
    s_hid5Cmd.endpoint = EP_IN;
    s_hid5Cmd.length = REPORT_SIZE;
    s_hid5Cmd.data = s_reportBuf;
    if (IOS_Ioctl(s_fd, HID5_INTERRUPT, &s_hid5Cmd, sizeof(s_hid5Cmd), s_reportBuf, REPORT_SIZE) <
            0) {
        SP_LOG("GCNAdapter: HID5 poll failed");
        s_connected = false;
        memset(s_ports, 0, sizeof(s_ports));
        return;
    }
    if (s_reportBuf[0] != REPORT_ID) {
        return;
    }
    parseReport(s_reportBuf);
}

static void hid5SendRumble(void) {
    s_rumbleBuf[0] = CMD_RUMBLE;
    for (u32 i = 0; i < GCN_PORT_COUNT; i++) {
        s_rumbleBuf[1 + i] = s_ports[i].rumble ? 0x01 : 0x00;
        s_ports[i].rumble = false;
    }
    memset(&s_hid5Cmd, 0, sizeof(s_hid5Cmd));
    s_hid5Cmd.deviceId = s_deviceId;
    s_hid5Cmd.endpoint = EP_OUT;
    s_hid5Cmd.length = 5;
    s_hid5Cmd.data = s_rumbleBuf;
    IOS_Ioctl(s_fd, HID5_INTERRUPT, &s_hid5Cmd, sizeof(s_hid5Cmd), NULL, 0);
}

// ---------------------------------------------------------------------------
// HID4 implementation (/dev/usb/hid, IOS37/53, older Wii)
// ---------------------------------------------------------------------------

static bool hid4Init(void) {
    s_fd = IOS_Open("/dev/usb/hid", 0);
    if (s_fd < 0) {
        return false;
    }

    // 1) Check version
    if (IOS_Ioctl(s_fd, HID4_GET_VERSION, NULL, 0, s_versionBuf, sizeof(s_versionBuf)) < 0) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }
    u32 ver = (u32)s_versionBuf[0] << 24 | (u32)s_versionBuf[1] << 16 | (u32)s_versionBuf[2] << 8 |
            (u32)s_versionBuf[3];
    if (ver != HID4_VERSION) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }

    // 2) Get device list
    if (IOS_Ioctl(s_fd, HID4_GET_DEVICE_CHANGE, NULL, 0, s_hid4DevChangeBuf,
                sizeof(s_hid4DevChangeBuf)) < 0) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }

    // 3) Find WUP-028 in the word array
    // HID4 device list is packed words: [length_words, device_id, vid_pid, ...]
    s_deviceId = 0;
    for (u32 i = 0;
            i < HID4_DEVICE_CHANGE_WORDS && s_hid4DevChangeBuf[i] < HID4_DEVICE_CHANGE_WORDS;) {
        u32 entryLen = s_hid4DevChangeBuf[i];
        if (entryLen == 0) {
            break;
        }
        u32 deviceId = s_hid4DevChangeBuf[i + 1];
        u32 vidPid = s_hid4DevChangeBuf[i + 2];
        u16 vid = (u16)(vidPid >> 16);
        u16 pid = (u16)(vidPid & 0xFFFF);
        if (vid == WUP028_VID && pid == WUP028_PID) {
            s_deviceId = deviceId;
            break;
        }
        i += entryLen / 4;
    }
    if (s_deviceId == 0) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }

    // 4) Send init command 0x13 via INTERRUPT_OUT
    s_cmdBuf[0] = CMD_INIT;
    memset(&s_hid4Cmd, 0, sizeof(s_hid4Cmd));
    s_hid4Cmd.deviceId = s_deviceId;
    s_hid4Cmd.endpoint = EP_OUT;
    s_hid4Cmd.length = 1;
    s_hid4Cmd.data = s_cmdBuf;
    if (IOS_Ioctl(s_fd, HID4_INTERRUPT_OUT, &s_hid4Cmd, sizeof(s_hid4Cmd), NULL, 0) < 0) {
        IOS_Close(s_fd);
        s_fd = -1;
        return false;
    }

    s_hidVersion = HID_VERSION_4;
    SP_LOG("GCNAdapter: HID4 init OK (id=%08x)", s_deviceId);
    return true;
}

static void hid4Poll(void) {
    memset(&s_hid4Cmd, 0, sizeof(s_hid4Cmd));
    s_hid4Cmd.deviceId = s_deviceId;
    s_hid4Cmd.endpoint = EP_IN;
    s_hid4Cmd.length = REPORT_SIZE;
    s_hid4Cmd.data = s_reportBuf;
    if (IOS_Ioctl(s_fd, HID4_INTERRUPT_IN, &s_hid4Cmd, sizeof(s_hid4Cmd), s_reportBuf,
                REPORT_SIZE) < 0) {
        SP_LOG("GCNAdapter: HID4 poll failed");
        s_connected = false;
        memset(s_ports, 0, sizeof(s_ports));
        return;
    }
    if (s_reportBuf[0] != REPORT_ID) {
        return;
    }
    parseReport(s_reportBuf);
}

static void hid4SendRumble(void) {
    s_rumbleBuf[0] = CMD_RUMBLE;
    for (u32 i = 0; i < GCN_PORT_COUNT; i++) {
        s_rumbleBuf[1 + i] = s_ports[i].rumble ? 0x01 : 0x00;
        s_ports[i].rumble = false;
    }
    memset(&s_hid4Cmd, 0, sizeof(s_hid4Cmd));
    s_hid4Cmd.deviceId = s_deviceId;
    s_hid4Cmd.endpoint = EP_OUT;
    s_hid4Cmd.length = 5;
    s_hid4Cmd.data = s_rumbleBuf;
    IOS_Ioctl(s_fd, HID4_INTERRUPT_OUT, &s_hid4Cmd, sizeof(s_hid4Cmd), NULL, 0);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void GCNAdapter_init(void) {
    memset(s_ports, 0, sizeof(s_ports));
    s_hidVersion = HID_NONE;
    s_connected = false;

    // Try HID5 first (vWii / IOS57+), fall back to HID4 (IOS37/53)
    if (hid5Init()) {
        s_connected = true;
        return;
    }
    if (hid4Init()) {
        s_connected = true;
        return;
    }

    SP_LOG("GCNAdapter: No adapter found on HID4 or HID5");
}

void GCNAdapter_poll(void) {
    if (!s_connected) {
        return;
    }
    if (s_hidVersion == HID_VERSION_5) {
        hid5Poll();
    } else if (s_hidVersion == HID_VERSION_4) {
        hid4Poll();
    }
}

void GCNAdapter_sendRumble(void) {
    if (!s_connected) {
        return;
    }
    if (s_hidVersion == HID_VERSION_5) {
        hid5SendRumble();
    } else if (s_hidVersion == HID_VERSION_4) {
        hid4SendRumble();
    }
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
