#include <stdlib.h>
#include <string.h>
#include "cec-frame.h"
#include "usb-cdc.h"
#include "cec-task.h"

#include "cec-log.h"

#include <stdio.h>
#include "tclie.h"
#include "pico/time.h"

// Helper: send CEC frame with timeout
static int send_cec_frame(uint8_t len, uint8_t *pld) {

    if (cec_log_enabled()) {
        cdc_printfln("[CEC] Attempting to send %d bytes...", len);
    }
    absolute_time_t start = get_absolute_time();
    const int timeout_ms = 1000;
    bool ack = false;
    while (absolute_time_diff_us(start, get_absolute_time()) < timeout_ms * 1000) {
        ack = cec_frame_send(len, pld);
        if (ack) break;
        sleep_ms(10);
    }
    if (cec_log_enabled()) {
        if (ack) {
            cdc_printfln("[CEC] Frame sent successfully.");
        } else {
            cdc_printfln("[CEC] Frame send failed (no ACK).");
        }
    }
    return ack ? 0 : -1;
}

// send raw XX XX ...
static int exec_send_raw(void *arg, int argc, const char **argv) {
    if (argc < 3) {
        cdc_printfln("Usage: send raw <hex byte1> <hex byte2> ...");
        return -1;
    }
    uint8_t pld[16] = {0};
    int pldcnt = argc - 2;
    if (pldcnt > 16) pldcnt = 16;
    for (int i = 0; i < pldcnt; ++i) {
        unsigned int val = 0;
        if (sscanf(argv[i + 2], "%x", &val) != 1 || val > 0xFF) {
            cdc_printfln("Invalid hex byte: %s", argv[i + 2]);
            return -1;
        }
        pld[i] = (uint8_t)val;
    }
    return send_cec_frame(pldcnt, pld);
}

// send image
static int exec_send_image(void *arg, int argc, const char **argv) {
    uint8_t pld[2] = { (cec_get_logical_address() << 4) | 0x00, 0x04 };
    return send_cec_frame(2, pld);
}

// send source id
static int exec_send_source(void *arg, int argc, const char **argv) {
    if (argc != 3) {
        cdc_printfln("Usage: send source <id>");
        return -1;
    }
    int id = atoi(argv[2]);
    if (id < 1 || id > 15) {
        cdc_printfln("Invalid source id: %d", id);
        return -1;
    }
    // Compose '4F 82 X0 00' where X is the source id
    uint8_t pld[4] = { 0x4F, 0x82, (uint8_t)(id << 4), 0x00 };
    return send_cec_frame(4, pld);
}

// send poweron
static int exec_send_poweron(void *arg, int argc, const char **argv) {
    uint8_t pld[3] = { (cec_get_logical_address() << 4) | 0x00, 0x44, 0x6D }; // User Control Pressed: Power On
    return send_cec_frame(3, pld);
}

// send standby
static int exec_send_standby(void *arg, int argc, const char **argv) {
    uint8_t pld[2] = { (cec_get_logical_address() << 4) | 0x00, 0x36 };
    return send_cec_frame(2, pld);
}

// send volup
static int exec_send_volup(void *arg, int argc, const char **argv) {
    uint8_t pld[3] = { (cec_get_logical_address() << 4) | 0x00, 0x44, 0x41 }; // User Control Pressed: Volume Up
    return send_cec_frame(3, pld);
}

// send voldown
static int exec_send_voldown(void *arg, int argc, const char **argv) {
    uint8_t pld[3] = { (cec_get_logical_address() << 4) | 0x00, 0x44, 0x42 }; // User Control Pressed: Volume Down
    return send_cec_frame(3, pld);
}

// send mute
static int exec_send_mute(void *arg, int argc, const char **argv) {
    uint8_t pld[3] = { (cec_get_logical_address() << 4) | 0x00, 0x44, 0x43 }; // User Control Pressed: Mute
    return send_cec_frame(3, pld);
}

// send vendorid
static int exec_send_vendorid(void *arg, int argc, const char **argv) {
    uint8_t pld[2] = { (cec_get_logical_address() << 4) | 0x00, 0x8C }; // Give Device Vendor ID
    int res = send_cec_frame(2, pld);

    if (res == 0) {
        // Wait for a response (blocking, adjust timeout as needed)
        uint8_t rx[16] = {0};
        uint8_t rxlen = cec_frame_recv(rx, cec_get_logical_address());
        if (rxlen >= 5 && rx[1] == 0x87) { // 0x87 = Device Vendor ID
            uint32_t vendor = (rx[2] << 16) | (rx[3] << 8) | rx[4];
            cdc_printfln("[CEC] Device Vendor ID: 0x%06lX", (unsigned long)vendor);
        } else {
            cdc_printfln("[CEC] No Device Vendor ID response.");
        }
    }
    return res;
}

// send reportpa
static int exec_send_reportpa(void *arg, int argc, const char **argv) {
    uint16_t paddr = cec_get_physical_address();
    uint8_t pld[5] = { (cec_get_logical_address() << 4) | 0x0F, 0x84, (paddr >> 8) & 0xFF, paddr & 0xFF, 0x04 };
    return send_cec_frame(5, pld);
}


const tclie_cmd_t cec_extra_cmds[] = {
    { "raw", exec_send_raw, "Send raw CEC bytes: send raw XX XX ...", NULL, { NULL, 0 } },
    { "standby", exec_send_standby, "Send standby command to TV", NULL, { NULL, 0 } },
    { "image", exec_send_image, "Send Image View On command to TV", NULL, { NULL, 0 } },
    { "source", exec_send_source, "Send Active Source command: send source <id> (id=1,2,3.. for input)", NULL, { NULL, 0 } },
    { "poweron", exec_send_poweron, "Send Power On command to TV", NULL, { NULL, 0 } },
    { "volup", exec_send_volup, "Send Volume Up command to TV", NULL, { NULL, 0 } },
    { "voldown", exec_send_voldown, "Send Volume Down command to TV", NULL, { NULL, 0 } },
    { "mute", exec_send_mute, "Send Mute command to TV", NULL, { NULL, 0 } },
    { "vendorid", exec_send_vendorid, "Request Device Vendor ID from TV", NULL, { NULL, 0 } },
    { "reportpa", exec_send_reportpa, "Broadcast Report Physical Address", NULL, { NULL, 0 } },
};



