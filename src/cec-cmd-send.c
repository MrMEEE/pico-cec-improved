// Add a new command to send CEC messages from the menu: "send (command) (parameters)"
// Usage: send <hex byte1> <hex byte2> ...
// Example: send 40 04 (sends [0x40, 0x04])

#include <stdlib.h>
#include <string.h>
#include "cec-frame.h"
#include "tclie.h"
#include "usb-cdc.h"

// Timeout version of cec_frame_send (non-blocking menu)

#include <stdio.h>
#include "pico/time.h"
#include "cec-cmd-extra.h" // for cec_extra_cmds and CEC_EXTRA_CMDS_COUNT

static void print_send_help(void) {
    cdc_printfln("send <raw|standby|image|source|poweron|volup|voldown|mute|vendorid|reportpa> ...");
    for (size_t i = 0; i < CEC_EXTRA_CMDS_COUNT; ++i) {
        cdc_printfln("  send %s - %s", cec_extra_cmds[i].name, cec_extra_cmds[i].desc);
    }
}

int exec_send_cec(void *arg, int argc, const char **argv) {
    if (argc < 2 || (argc == 2 && strcmp(argv[1], "help") == 0)) {
        print_send_help();
        return 0;
    }
    // Dispatch to subcommands
    for (size_t i = 0; i < CEC_EXTRA_CMDS_COUNT; ++i) {
        if (strcmp(argv[1], cec_extra_cmds[i].name) == 0) {
            return cec_extra_cmds[i].fn(arg, argc, argv);
        }
    }
    cdc_printfln("Unknown send subcommand: %s", argv[1]);
    print_send_help();
    return -1;
}

// Reference extra send commands as subcommands

const tclie_cmd_t send_command = {
    .name = "send",
    .fn = exec_send_cec,
    .desc = "Send CEC message.",
    .pattern = NULL,
    .options = { (const tclie_cmd_opt_t *)NULL, 0 },
};
