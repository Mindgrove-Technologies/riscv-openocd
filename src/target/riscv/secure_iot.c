/*
 * secureiot.c — secureiot command registration hub
 */


#include "secure_iot_common.h"


/* ---- secureiot command leaf table ---- */
const struct command_registration secureiot_command_handlers[] = {

    {
        .name = "psram_init",
        .handler = handle_psram_init,
        .mode = COMMAND_EXEC,
        .help = "Initialize PSRAM",
        .usage = "<0|1>",
    },

    {
        .name = "flash_erase",
        .handler = handle_flash_erase,
        .mode = COMMAND_EXEC,
        .help = "Flash erase",
        .usage = "<sector>",
    },

    {
        .name = "flash_write",
        .handler = handle_flash_write,
        .mode = COMMAND_EXEC,
        .help = "Flash write",
    },

    {
        .name = "flash_xip_init",
        .handler = handle_flash_xip,
        .mode = COMMAND_EXEC,
        .help = "Enable flash XIP",
    },

    {
        .name = "flash_sector_erase",
        .handler = handle_sector_erase,
        .mode = COMMAND_EXEC,
        .help = "Erase flash sector",
    },

    {
        .name = "reset",
        .handler = handle_reset,
        .mode = COMMAND_EXEC,
        .help = "Reset secureiot",
    },

    {
        .name = "flash_write_length",
        .handler = handle_flash_write_length,
        .mode = COMMAND_EXEC,
    },

    {
        .name = "flash_write_data",
        .handler = handle_flash_write_data,
        .mode = COMMAND_EXEC,
    },

    COMMAND_REGISTRATION_DONE
};


int secureiot_register_commands(struct command_context *cmd_ctx)
{
    return register_commands(cmd_ctx, NULL, secureiot_command_handlers);
}
