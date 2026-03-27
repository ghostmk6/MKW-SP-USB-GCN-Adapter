#pragma once

#include <Common.h>
#include <stdbool.h>

#define GCN_ADAPTER_VID 0x057E
#define GCN_ADAPTER_PID 0x0337
#define GCN_PORT_COUNT 4

typedef struct {
    bool connected;
    bool wireless;
    bool a, b, x, y;
    bool l, r, z, start;
    bool dpadUp, dpadDown, dpadLeft, dpadRight;
    u8 stickX, stickY;
    u8 cstickX, cstickY;
    u8 triggerL, triggerR;
    bool rumble;
} GCNPortState;

void GCNAdapter_init(void);
void GCNAdapter_poll(void);
void GCNAdapter_sendRumble(void);
bool GCNAdapter_isConnected(void);
const GCNPortState *GCNAdapter_getPort(u32 port);
