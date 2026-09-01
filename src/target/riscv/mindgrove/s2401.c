
#include "s2401_common.h"


/* ---- s2401 command leaf table ---- */
const struct command_registration s2401_command_handlers[] = {

    {
        .name = "psram_init",
        .handler = s2401_handle_psram_init,
        .mode = COMMAND_EXEC,
        .help = "Initialize PSRAM",
        .usage = "<instance>",
    },

    {
        .name = "flash_erase",
        .handler = s2401_handle_flash_erase,
        .mode = COMMAND_EXEC,
        .help = "Flash erase",
        .usage = "<instance>",
    },

    {
        .name = "flash_write",
        .handler = s2401_handle_flash_write,
        .mode = COMMAND_EXEC,
        .help = "Flash write",
        .usage = "<file> <address>",
    },

    {
        .name = "flash_xip_init",
        .handler = s2401_handle_flash_xip,
        .mode = COMMAND_EXEC,
        .help = "Enable flash XIP",
        .usage = "<instance>",
    },

    {
        .name = "flash_sector_erase",
        .handler = s2401_handle_sector_erase,
        .mode = COMMAND_EXEC,
        .help = "Erase flash sector",
        .usage = "<instance> <address> <num_sectors>",

    },

    {
        .name = "reset",
        .handler = s2401_handle_reset,
        .mode = COMMAND_EXEC,
        .help = "Reset s2401",
        .usage = "",
    },

    {
        .name = "flash_write_length",
        .handler = s2401_handle_flash_write_length,
        .mode = COMMAND_EXEC,
        .usage = "<instance> <file> <address>",
    },

    {
        .name = "flash_write_data",
        .handler = s2401_handle_flash_write_data,
        .mode = COMMAND_EXEC,
        .usage = "<address> <data>",
    },

    COMMAND_REGISTRATION_DONE
};


int s2401_register_commands(struct command_context *cmd_ctx)
{
    return register_commands(cmd_ctx, NULL, s2401_command_handlers);
}
