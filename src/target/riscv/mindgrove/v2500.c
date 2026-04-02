
#include "v2500_common.h"


/* ---- v2500 command leaf table ---- */
const struct command_registration v2500_command_handlers[] = {

    // {
    //     .name = "psram_init",
    //     .handler = v2500_handle_psram_init,
    //     .mode = COMMAND_EXEC,
    //     .help = "Initialize PSRAM",
    //     .usage = "Ram mode",
    // },

    {
        .name = "flash_erase",
        .handler = v2500_handle_flash_erase,
        .mode = COMMAND_EXEC,
        .help = "Flash erase",
        .usage = "<instance>",
    },

    {
        .name = "flash_write",
        .handler = v2500_handle_flash_write,
        .mode = COMMAND_EXEC,
        .help = "Flash write",
        .usage = "<file> <address>",
    },

    {
        .name = "flash_xip_init",
        .handler = v2500_handle_flash_xip,
        .mode = COMMAND_EXEC,
        .help = "Enable flash XIP",
    },
    {
        .name = "reset",
        .handler = v2500_handle_reset,
        .mode = COMMAND_EXEC,
        .help = "Reset v2500",
        .usage = "",
    },
    // {
    //     .name = "flash_sector_erase",
    //     .handler = v2500_handle_sector_erase,
    //     .mode = COMMAND_EXEC,
    //     .help = "Erase flash sector",
    // },

    // {
    //     .name = "flash_write_length",
    //     .handler = v2500_handle_flash_write_length,
    //     .mode = COMMAND_EXEC,
    // },

    // {
    //     .name = "flash_write_data",
    //     .handler = v2500_handle_flash_write_data,
    //     .mode = COMMAND_EXEC,
    // },

    COMMAND_REGISTRATION_DONE
};


int v2500_register_commands(struct command_context *cmd_ctx)
{
    return register_commands(cmd_ctx, NULL, v2500_command_handlers);
}
